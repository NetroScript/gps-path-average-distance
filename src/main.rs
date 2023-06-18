use std::path::PathBuf;
use std::io::BufReader;
use std::fs::File;

use gpx::{read, Waypoint, write};
use gpx::{Gpx, Track, TrackSegment};
use geo::{ClosestPoint,Closest,Simplify, LineString};
use geo::GeodesicDistance;
use clap::{arg, command, Parser};
use colored::{Colorize, control};


#[derive(Parser)]
#[command(name = "gps-path-average-distance", version = "1.0", author = "NetroScript", about = "This application takes a reference GPS path and compares it to one to many other gps tracks to then calculate the average distance between the two paths. This works by averaging the distance of the closest point on the reference path to the other path.")]
struct Cli {

    /// File path to a .gpx file containing the reference path
    #[arg(short, long, required = true)]
    reference: PathBuf,

    /// One to multiple file paths to a .gpx file containing a track to compare to the reference path. Separate multiple paths with a comma.
    #[arg(short, long, required = true, value_delimiter = ',', num_args = 1)]
    track: Vec<PathBuf>,

    /// Turn debugging information on
    #[arg(short, long, action = clap::ArgAction::Count)]
    debug: u8,

    /// Custom epsilon value to use for simplifying the reference path. This is the maximum distance between two points before they are simplified. Default is 0.0001.
    #[arg(short, long, default_value = "0.00001")]
    simplify_epsilon: f64,

    /// Toggle to also reexport the parsed GPX files as simplified GPX files
    #[arg(short, long)]
    export_track: bool,
}

// Add a macro to print out the debug information
// Make the color green and prefix it with [DEBUG]
macro_rules! debug_print {
    ($debug_enabled:expr, $($arg:tt)*) => {
        if $debug_enabled {
            println!("{} {}", "[DEBUG]".green(), format!($($arg)*));
        }
    }
}


fn main() {


    let _ = control::set_virtual_terminal(true);

    // Parse the command line arguments
    let matches = Cli::parse();


    let reference_path: PathBuf = matches.reference;

    // Generate a path buffer from the input strings
    let track_paths: Vec<PathBuf> = matches.track.iter().map(|s| PathBuf::from(s)).collect();
    //let track_paths: Vec<PathBuf> = matches.get_many::<String>("track").expect("At least one track is required so it can be compared against the reference").map(|s| PathBuf::from(s)).collect();

    let debug_enabled: bool = matches.debug > 0;


    debug_print!(debug_enabled, "Debugging is enabled");
    debug_print!(debug_enabled, "Reference path: {:?}", reference_path);
    debug_print!(debug_enabled, "Track paths: {:?}", track_paths);
    debug_print!(debug_enabled, "Simplify epsilon: {}", matches.simplify_epsilon);

    // Check that all passed paths exist and are files
    if !reference_path.exists() {
        println!("The reference path {:?} does not exist", reference_path);
        return;
    }
    if !reference_path.is_file() {
        println!("The reference path {:?} is not a file", reference_path);
        return;
    }

    for track_path in &track_paths {
        if !track_path.exists() {
            println!("The track path {:?} does not exist", track_path);
            return;
        }
        if !track_path.is_file() {
            println!("The track path {:?} is not a file", track_path);
            return;
        }
    }

    // Read in the reference path as a GPX file
    let reference_file = File::open(reference_path).expect("Failed to open reference path");
    let reference_reader = BufReader::new(reference_file);
    let reference_gpx: Gpx = read(reference_reader).expect("Failed to read reference path as GPX");

    // Read in the track paths as GPX files
    let mut track_gpxs: Vec<Gpx> = Vec::new();

    for track_path in &track_paths {
        let track_file = File::open(track_path).expect("Failed to open track path");
        let track_reader = BufReader::new(track_file);
        let track_gpx: Gpx = read(track_reader).expect("Failed to read track path as GPX");
        track_gpxs.push(track_gpx);
    }

    // Get the first track of the reference path
    let reference_track: &Track = &reference_gpx.tracks[0];

    // Get the total number of tracks by iterating all the track GPXs and summing the number of tracks
    let total_tracks: usize = track_gpxs.iter().map(|gpx| gpx.tracks.len()).sum();
    
    println!("Calculating average distance between reference path ({}) and {} track(s)... ", reference_track.name.as_ref().unwrap_or(&"-- Unnamed --".to_string()).italic(), total_tracks);

    // Keep track of the current index of a track
    let mut track_index: usize = 0;



    // Iterate every track now
    for (gpx_index,track_gpx) in track_gpxs.iter().enumerate() {

        // Create a copy of the gpx file so we can modify it and  reexport it if needed
        let mut track_gpx_copy: Gpx = track_gpx.clone();


        for (cur_track_index, track) in track_gpx.tracks.iter().enumerate() {
            println!("Track {}: {}", track_index + 1, track.name.as_ref().unwrap_or(&"-- Unnamed --".to_string()));

            let mut total_distance: f64 = 0.0;
            let mut total_points: usize = 0;
            let mut total_distance_simplified: f64 = 0.0;
            let mut total_points_simplified: usize = 0;

            // If we want to reexport the GPX files, we need to clear the track segments
            if matches.export_track {
                track_gpx_copy.tracks[cur_track_index].segments.clear();
            }

            // Iterate every segment of the track
            track.segments.iter().for_each(|segment| {

                // Inline function to calculate the total distance of a segment
                let calculate_segment_distance = |line_string: &LineString, distance: &mut f64, points: &mut usize| {
                    // For every point in the segment, find the closest point on the reference path
                    line_string.points().for_each(|point| {
                        
                        // By default, set the closest distance to infinity
                        let mut closest_distance: f64 = f64::INFINITY;

                        // Find the distance to the closest point on the reference path
                        reference_track.segments.iter().for_each(|reference_segment| {

                            let closest = reference_segment.linestring().closest_point(&point);
                            let distance = match closest {
                                Closest::Intersection(p) => p.geodesic_distance(&point),
                                Closest::Indeterminate => f64::INFINITY,
                                Closest::SinglePoint(p) => p.geodesic_distance(&point),
                            };

                            if distance < closest_distance {
                                closest_distance = distance;
                            }
                        });

                        // Add the closest distance to the total distance
                        *distance += closest_distance;
                        *points += 1;
                    });
                };

                // Calculate the average distance of the segment just based on the points
                calculate_segment_distance(&segment.linestring(), &mut total_distance, &mut total_points);
                // Also create a simplified version of the linestring, where points that are close to each other are removed to have a more accurate average distance
                let simplified_linestring: LineString = segment.linestring().simplify(&matches.simplify_epsilon);

                debug_print!(debug_enabled, "Original points: {:?} - Simplified points: {:?}", segment.linestring().points().count(), simplified_linestring.points().count());

                // If we want to export the simplified linestring, do it now
                if matches.export_track {
                    let mut simplified_segment: TrackSegment = TrackSegment::new();
                    simplified_segment.points = simplified_linestring.points().map(|point| Waypoint::new(point)).collect();
                    track_gpx_copy.tracks[cur_track_index].segments.push(simplified_segment);
                }

                // Calculate the total distance of the simplified linestring
                calculate_segment_distance(&simplified_linestring, &mut total_distance_simplified, &mut total_points_simplified);
            });

            println!("Average distance (in time): {} (counting every point)", (format!("{:.3}m", (total_distance / total_points as f64))).cyan().bold());
            println!("Average distance (location dependant): {} (counting only simplified points)", (format!("{:.3}m", (total_distance_simplified / total_points_simplified as f64))).yellow().bold());

             
            track_index += 1;
        }


        // If we want to reexport the GPX files, do it now by writing the modified GPX file to the same path, adding .modified before the extension
        if matches.export_track {
            let track_path = track_paths[gpx_index].clone();
            let mut modified_path = track_path.clone();
            modified_path.set_extension("modified.gpx");
            let track_file = File::create(&modified_path).expect("Failed to create modified track file");
            write(&track_gpx_copy, track_file).expect("Failed to write modified track file");
            println!("Exported modified track file to {:?}", &modified_path);
        }
    }

}
