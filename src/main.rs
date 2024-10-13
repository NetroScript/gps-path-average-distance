use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;
use std::process;

use clap::{arg, Parser};
use colored::Colorize;
use flat_projection::{FlatPoint, FlatProjection};
use geo::{Closest, ClosestPoint, Coord, LineString, Point, Simplify};
use geo::{EuclideanDistance, FrechetDistance, HausdorffDistance};
use gpx::{read, write, Waypoint};
use gpx::{Gpx, Track, TrackSegment};

#[derive(Parser)]
#[command(
    name = "gps-path-average-distance",
    version = "0.1.3",
    author = "NetroScript",
    about = "This application compares a reference GPS path to other tracks by calculating four distances: point-wise average distance, simplified point-wise average distance, Fréchet distance, and Hausdorff distance."
)]
struct Cli {
    /// File path to a .gpx file containing the reference path
    #[arg(short, long, required = true)]
    reference: PathBuf,

    /// One to multiple file paths to a .gpx file containing a track to compare to the reference path. Separate multiple paths with a comma.
    #[arg(short, long, required = true, value_delimiter = ',', num_args = 1)]
    track: Vec<PathBuf>,

    /// Turn debugging information on
    #[arg(short, long)]
    debug: bool,

    /// Custom epsilon value to use for simplifying the reference path. This is the maximum distance between two points before they are simplified. The value is given in meters. Default is 1m.
    #[arg(short, long, default_value = "1.0")]
    simplify_epsilon: f64,

    /// Toggle to also reexport the parsed GPX files as simplified GPX files
    #[arg(short, long)]
    export_track: bool,

    /// Toggle to only output JSON data in the console
    #[arg(short, long)]
    json: bool,
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

// Macro to only print if the json flag is not set
macro_rules! print_info {
    ($json_enabled:expr, $($arg:tt)*) => {
        if !$json_enabled {
            println!($($arg)*);
        }
    }
}

fn main() {
    #[cfg(windows)]
    {
        let _ = colored::control::set_virtual_terminal(true).unwrap_or(());
    }

    // Parse the command line arguments
    let matches = Cli::parse();

    // Error and exit if both debug and json are enabled
    if matches.debug && matches.json {
        eprintln!("Both debug and json flags are enabled. Please only enable one of them.");
        process::exit(1);
    }

    let reference_path: PathBuf = matches.reference;

    // Generate a path buffer from the input strings
    let track_paths: Vec<PathBuf> = matches.track.iter().map(|s| PathBuf::from(s)).collect();

    debug_print!(matches.debug, "Debugging is enabled");
    debug_print!(matches.debug, "Reference path: {:?}", reference_path);
    debug_print!(matches.debug, "Track paths: {:?}", track_paths);
    debug_print!(
        matches.debug,
        "Simplify epsilon: {}",
        matches.simplify_epsilon
    );

    // Check that all passed paths exist and are files
    if !reference_path.exists() {
        eprintln!("The reference path {:?} does not exist", reference_path);
        process::exit(1);
    }
    if !reference_path.is_file() {
        eprintln!("The reference path {:?} is not a file", reference_path);
        process::exit(1);
    }

    for track_path in &track_paths {
        if !track_path.exists() {
            eprintln!("The track path {:?} does not exist", track_path);
            process::exit(1);
        }
        if !track_path.is_file() {
            eprintln!("The track path {:?} is not a file", track_path);
            process::exit(1);
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

    // Check that the reference path has at least one track
    let reference_track: Track = if reference_gpx.tracks.len() > 0 {
        // If there are more than 1 track, print a warning that only the first track will be used and that it potentially should be manually checked if this is correct
        if reference_gpx.tracks.len() > 1 {
            print_info!(matches.json, "The reference path contains more than one track. Only the first track will be used. Please verify that this is the correct track.");
        }

        // Get the first track of the reference path
        reference_gpx.tracks[0].clone()
    }
    // Check if the reference path has any waypoints
    // If so create a Track with a single TrackSegment containing all the waypoints
    else if reference_gpx.waypoints.len() > 0 {
        print_info!(matches.json, "The reference path does not contain any tracks, but it does contain waypoints. Creating a track from the waypoints");
        let mut track: Track = Track::default();
        let mut track_segment: TrackSegment = TrackSegment::default();
        track_segment.points = reference_gpx.waypoints.clone();
        track.segments.push(track_segment);
        track.clone()
    } else {
        // No waypoints or tracks so we exit the program
        eprintln!("The reference path does not contain any tracks or waypoints");
        process::exit(1)
    };

    // Get the total number of tracks by iterating all the track GPXs and summing the number of tracks
    let total_tracks: usize = track_gpxs.iter().map(|gpx| gpx.tracks.len()).sum();

    print_info!(
        matches.json,
        "Calculating average distance between reference path ({}) and {} track(s)... ",
        reference_track
            .name
            .as_ref()
            .unwrap_or(&"-- Unnamed --".to_string())
            .italic(),
        total_tracks
    );

    // Keep track of the current index of a track
    let mut track_index: usize = 0;

    // Iterate every track now
    for (gpx_index, track_gpx) in track_gpxs.iter().enumerate() {
        // Create a copy of the gpx file so we can modify it and  reexport it if needed
        let mut track_gpx_copy: Gpx = track_gpx.clone();

        for (cur_track_index, track) in track_gpx.tracks.iter().enumerate() {
            print_info!(
                matches.json,
                "Track {}: {}",
                track_index + 1,
                track.name.as_ref().unwrap_or(&"-- Unnamed --".to_string())
            );

            // Tracks may contain multiple segments, we however assume that there is only one segment, thus in
            // Files with multiple segments, we combine them into a single LineString which is then used for the calculations

            // Additionally, GPS tracks are stored in LatLon coordinates, which are not suitable for distance calculations
            // as the distance in meter varies depending on the latitude (1° latitude ranges from ~111 km at the equator to 0 km at the poles)
            // To solve this, we project the coordinates to a flat coordinate system
            // This is "very precise" for distances of up to about 500km

            // To do so, find the average position of all the points in the reference track, around which we can project the coordinates
            let total_points = reference_track
                .segments
                .iter()
                .map(|segment| segment.points.len() as f64)
                .sum::<f64>();
            let sum_positions = reference_track
                .segments
                .iter()
                .flat_map(|segment| &segment.points)
                .fold(Point::new(0.0, 0.0), |acc, waypoint| {
                    Point::new(
                        acc.x() + waypoint.point().x(),
                        acc.y() + waypoint.point().y(),
                    )
                });
            let average_position = Point::new(
                sum_positions.x() / total_points,
                sum_positions.y() / total_points,
            );

            let projector = FlatProjection::new(average_position.x(), average_position.y());

            /// Function to join segments and project them into a flat coordinate system.
            /// This function takes a list of segments, projects their points, and returns a single LineString.
            /// - `segments`: The GPS track segments to be joined and projected.
            /// - `projector`: The flat coordinate system used for projection.
            /// - Returns: A LineString containing all the projected points.
            fn join_and_project_segments(
                segments: &[TrackSegment],
                projector: &FlatProjection<f64>,
            ) -> LineString {
                let mut joined_segment = TrackSegment::new();
                segments.iter().for_each(|segment| {
                    joined_segment
                        .points
                        .extend(segment.points.iter().map(|point| {
                            let projected_point =
                                projector.project(point.point().x(), point.point().y());
                            Waypoint::new(Point::new(projected_point.x, projected_point.y))
                        }));
                });
                joined_segment.linestring()
            }

            /// Function to unproject a LineString from a flat coordinate system back to LatLon coordinates.
            /// This function takes a LineString in a flat coordinate system and unprojects the points back to LatLon coordinates.
            /// - `linestring`: The LineString to be unprojected.
            /// - `projector`: The flat coordinate system used for projection.
            /// - Returns: A LineString containing all the unprojected points.
            fn unproject_linestring(
                linestring: &LineString,
                projector: &FlatProjection<f64>,
            ) -> LineString {
                linestring
                    .points()
                    .map(|point| {
                        let unprojected_point = projector.unproject(&FlatPoint {
                            x: point.x(),
                            y: point.y(),
                        });
                        Coord {
                            x: unprojected_point.0,
                            y: unprojected_point.1,
                        }
                    })
                    .collect()
            }

            /// Function to calculate the total length of a LineString.
            /// The length is computed as the sum of distances between consecutive points in the LineString.
            /// - `linestring`: The LineString whose total length is to be calculated.
            /// - Returns: The total length of the LineString in kilometers.
            fn calculate_total_length(linestring: &LineString) -> f64 {
                linestring
                    .points()
                    .zip(linestring.points().skip(1))
                    .map(|(p1, p2)| p1.euclidean_distance(&p2))
                    .sum()
            }

            // Use the function to join and project both the current and reference tracks.
            let joined_current_linestring = join_and_project_segments(&track.segments, &projector);
            let joined_reference_linestring =
                join_and_project_segments(&reference_track.segments, &projector);

            // Calculate the total length of the joined LineStrings for both the current and reference tracks.
            let current_track_length = calculate_total_length(&joined_current_linestring);
            let reference_track_length = calculate_total_length(&joined_reference_linestring);

            // If either track length is above 500km, print a warning that the distance may not be as precise
            if current_track_length > 500.0 || reference_track_length > 500.0 {
                print_info!(matches.json,
                    "Warning: The total length of the current track is {} km and the total length of the reference track is {} km. The distance computations may not be as precise due to using a fast flat projection.",
                    format!("{:.3}", current_track_length).red().bold(),
                    format!("{:.3}", reference_track_length).red().bold()
                );
            }

            // Calculate the frechet distance (in kilometers)
            let frechet_distance =
                joined_current_linestring.frechet_distance(&joined_reference_linestring);

            // Calculate the hausdorff distance (in kilometers)
            let hausdorff_distance =
                joined_current_linestring.hausdorff_distance(&joined_reference_linestring);

            let mut total_distance: f64 = 0.0;
            let mut total_points: usize = 0;
            let mut total_distance_simplified: f64 = 0.0;
            let mut total_points_simplified: usize = 0;

            // If we want to reexport the GPX files, we need to clear the track segments
            if matches.export_track {
                track_gpx_copy.tracks[cur_track_index].segments.clear();
            }

            // Next we want to compute the "average" and the "simplified average" distance between the reference path and the current track
            // The average distance is computed by taking every point of the current track and finding the closest point on the reference path, then calculating the distance between them and summing them up divided by the total number of points
            // For the simplified average distance, we first simplify the reference path by removing points that are closer than a certain epsilon value to each other using the Ramer-Douglas-Peucker algorithm and then do the same as for the average distance

            // Function to calculate the average distance between two LineStrings.
            let calculate_average_distance =
                |current_linestring: &LineString,
                 reference_linestring: &LineString,
                 distance: &mut f64,
                 points: &mut usize| {
                    current_linestring.points().for_each(|point| {
                        // Find the distance to the closest point on the reference path
                        let closest_point = reference_linestring.closest_point(&point);

                        let current_distance = match closest_point {
                            Closest::Intersection(p) => p.euclidean_distance(&point),
                            Closest::Indeterminate => f64::INFINITY,
                            Closest::SinglePoint(p) => p.euclidean_distance(&point),
                        };

                        // Add the distance to the total distance
                        *distance += current_distance;
                        *points += 1;
                    });
                };

            // First we create a simplified version of the reference path
            let simplified_linestring: LineString =
                joined_current_linestring.simplify(&(matches.simplify_epsilon / 1000.0));

            // If we want to reexport the GPX files, we need to add the simplified LineString to the track segments
            // For this we need to convert the flat coordinates back to LatLon coordinates
            if matches.export_track {
                let simplified_unprojected_linestring =
                    unproject_linestring(&simplified_linestring, &projector);

                let mut track_segment = TrackSegment::new();
                track_segment.points = simplified_unprojected_linestring
                    .points()
                    .map(|point| Waypoint::new(point))
                    .collect();
                track_gpx_copy.tracks[cur_track_index]
                    .segments
                    .push(track_segment);
            }

            // Calculate the average distance between the current and reference tracks
            calculate_average_distance(
                &joined_current_linestring,
                &joined_reference_linestring,
                &mut total_distance,
                &mut total_points,
            );

            // Calculate the average distance between the simplified current and reference tracks
            calculate_average_distance(
                &simplified_linestring,
                &joined_reference_linestring,
                &mut total_distance_simplified,
                &mut total_points_simplified,
            );

            if matches.json {
                // Construct a JSON object and print it
                let json_output = serde_json::json!({
                    "track_index": cur_track_index + 1,
                    "track_name": track.name.as_ref().unwrap_or(&"-- Unnamed --".to_string()),
                    "current_track_length_m": current_track_length * 1000.0,
                    "reference_track_length_m": reference_track_length * 1000.0,
                    "average_distance_m": (total_distance / total_points as f64) * 1000.0,
                    "simplified_average_distance_m": (total_distance_simplified / total_points_simplified as f64) * 1000.0,
                    "frechet_distance_m": frechet_distance * 1000.0,
                    "hausdorff_distance_m": hausdorff_distance * 1000.0,
                });

                // Print the JSON object
                println!("{}", json_output.to_string());
            } else {
                // Print the lengths of the tracks
                println!(
                    "Total length of current track: {}",
                    format!("{:.3}m", current_track_length * 1000.0).bold()
                );
                println!(
                    "Total length of reference track: {}",
                    format!("{:.3}m", reference_track_length * 1000.0).bold()
                );
                println!(
                    "Average distance (in time): {} (counting every point)",
                    (format!("{:.3}m", (total_distance / total_points as f64) * 1000.0))
                        .cyan()
                        .bold()
                );
                println!(
                    "Average distance (location dependent): {} (counting only simplified points)",
                    (format!(
                        "{:.3}m",
                        (total_distance_simplified / total_points_simplified as f64) * 1000.0
                    ))
                    .yellow()
                    .bold()
                );
                println!(
                    "Fréchet distance: {}",
                    (format!("{:.3}m", frechet_distance * 1000.0))
                        .magenta()
                        .bold()
                );
                println!(
                    "Hausdorff distance: {}",
                    (format!("{:.3}m", hausdorff_distance * 1000.0))
                        .green()
                        .bold()
                );
            }

            track_index += 1;
        }

        // If we want to reexport the GPX files, do it now by writing the modified GPX file to the same path, adding .modified before the extension
        if matches.export_track {
            let track_path = track_paths[gpx_index].clone();
            let mut modified_path = track_path.clone();
            modified_path.set_extension("modified.gpx");
            let track_file =
                File::create(&modified_path).expect("Failed to create modified track file");
            write(&track_gpx_copy, track_file).expect("Failed to write modified track file");
            println!("Exported modified track file to {:?}", &modified_path);
        }
    }
}
