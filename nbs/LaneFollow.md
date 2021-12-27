# Lane Follow

## Multiple Approaches

1. Perspective Wrapped + Sliding Window Search
1. Normal ROI Masking + Sliding Window Search
1. Perspective Wrapped + Hough Lines Search
1. Normal ROI Masking + Hough Lines Search

### Perspective Wrapping

Function Pipeline :- Normal Image -> Perspective Wrap -> Process Image -> Final Output (Canny Edge Detected Image or Gray Thresholded Image)

### Normal ROI Masking

Function Pipeline:- Normal Image -> ROI Function -> Process Image -> Final Output (Canny Edge Detected Image or Gray Thresholded Image)

### Sliding Window Search

Function Pipeline:- Canny Edge Detected Image or Gray Thresholded Image -> Histogram

Funciton Pipeline:- Canny Edge Detected Image or Gray Thresholded Image -> Sliding Window Search(Image,Histogram)-> Compute Steering Angle (Lane Lines Slope)

### Hough Lines Search (Probably Faster)

Canny Edge Detected Image or Gray Thresholded Image -> Find Lanes -> Average Slope Intercept -> Compute Steering Angle (Lane Lines Slope)

### Hough Lines Search (Other Way)

Canny Edge Detected Image or Gray Thresholded Image -> Find Lanes -> Average Slope Intercept -> Compute Steering Angle (Lane Lines Coord)
