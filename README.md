# Automated Discontinuity Set Extraction from 3D Point Clouds

This MATLAB implementation provides an automated approach for identifying structural discontinuity sets and extracting individual discontinuity planes from 3D laser-scanned point clouds of rock surfaces.

The workflow includes normal estimation, dip angle and dip direction calculation, kernel density estimation (KDE) on a stereonet for discontinuity set identification, and connected components segmentation for extracting individual discontinuity planes.

## Repository Files

- **`DiscontinuitySetExtractor.m`** – Main MATLAB script to execute the complete workflow.
- **`Stereonet_OwnFigureCode_FascillaAlternative.m`** – Supporting script for stereonet generation and visualisation.
- **`affine_fit.m`** – Supporting function for least-squares plane fitting.
- **Example Data - Cloud`.las` file** – Sample 3D point cloud provided for testing.

## How to Run

1. Download or clone the repository.
2. Open MATLAB and set the Current Folder to the repository directory.
3. Ensure all three MATLAB files are present in the same directory.
4. **Run `DiscontinuitySetExtractor.m`.**
5. When prompted, select the provided example `.las` file or another suitable LAS point cloud.
6. The script automatically processes the point cloud and generates the visualisations.

## Output

The script generates intermediate visualisations of point-wise orientations, dip angle, dip direction, and KDE distributions.

The two main outputs are:

1. **Detected Discontinuity Sets:** Identified discontinuity sets visualised on a stereonet, with unique set IDs and colours.
2. **Identified Discontinuities:** Extracted discontinuity planes visualised in 3D, colour-coded according to their corresponding discontinuity set IDs.

Both main figures are automatically saved as **JPG images** in the same directory as the main MATLAB script:

- `Detected Discontinuity Sets.jpg`
- `Identified Discontinuities Coloured by Set.jpg`

## Requirements

MATLAB with the required toolboxes for LiDAR point-cloud processing, image processing, and statistical analysis.

## Notes

Processing parameters, including point-cloud downsampling resolution, KDE bandwidth, discontinuity set detection thresholds, connected components voxel size, and minimum component size, can be adjusted within the main script depending on the dataset.
