# GPS Path Average Distance

This Rust application is designed to compute the average distance between a reference GPS path and one or more additional GPS tracks. The calculation is based on determining the closest point on the reference path for each point on the track(s) being compared.

The application provides two output values: an average distance "*in time*"* and an "*location-dependent*" average distance. The rationale behind this distinction is as follows:

When sampling points along the track(s) to compare against the reference, it is possible to encounter multiple points at the same location if a person remains stationary for a period of time. Consequently, the longer the stationary period, the more the distance measurement becomes skewed towards that particular location. However, the focus may be on understanding the actual differences in paths, irrespective of movement speed or time. To address this, the algorithm simplifies the track using the Douglas-Peucker algorithm prior to calculating the average distance. This simplification process retains only points corresponding to changes in the path's direction, resulting in a simplified path. The average distance is then computed based on this simplified representation, which more accurately reflects the average distance between the paths, independent of movement speed or time.

## Table of Contents

- [GPS Path Average Distance](#gps-path-average-distance)
  - [Table of Contents](#table-of-contents)
  - [Installation / Building from Source](#installation--building-from-source)
  - [Usage](#usage)
  - [Options](#options)
  - [Example](#example)
  - [Exemplary Output](#exemplary-output)

## Installation / Building from Source

1. Install Rust and Cargo from [https://www.rust-lang.org](https://www.rust-lang.org).
2. Clone the repository or download the source code.
3. Open a terminal and navigate to the project directory.
4. Run the command `cargo build --release` to compile the application.


## Usage

This assumes you have a compiled version of the application.

The application takes input in the form of GPX files (.gpx) for both the reference path and the tracks to compare. Here's how to use it:

```shell
gps-path-average-distance [OPTIONS] --reference <REFERENCE> --track <TRACK>
```

## Options

Options
* `-r, --reference <REFERENCE>`: File path to the .gpx file containing the reference path.
* `-t, --track <TRACK>`: File path(s) to the .gpx file(s) containing the track(s) to compare. Separate multiple paths with a comma.
* `-d, --debug`: Turn on debugging information.
* `-s, --simplify_epsilon <EPSILON>`: Custom epsilon value for simplifying the reference path. Default is 0.00001.
    * This value is used as the epsilon in the Douglas-Peucker algorithm for simplifying the current path. The bigger the value, the more simplified the path will be. The reference "space" is in latitude and longitude, for this reason the value is quite small by default.
* `-e, --export_track`: Toggle to also reexport the parsed GPX files as simplified GPX files.
    * Exported files will be named `<original_file_name>.modified.gpx` and will be placed in the same directory as the original file. No GPX extensions are supported, so you will be left with only track points containing latitude and longitude.

5. Replace `[arguments]` with the appropriate command-line arguments. Here are the available options:

## Example

```shell
gps-path-average-distance -r ./reference.gpx -t ./track1.gpx,./track2.gpx -d -e
```

## Exemplary Output

The following figure shows four example paths.
 The red path is the reference path, while the blue and green paths are the tracks to compare.
 (Green is the original track, while Blue is the simplified track). 
 You can see how there is almost no visual difference between the original path and the simplified path.

![Path-distances-example](https://github.com/NetroScript/gps-path-average-distance/assets/18115780/4da5712b-aa1b-4eae-8f52-006a39bccdee)