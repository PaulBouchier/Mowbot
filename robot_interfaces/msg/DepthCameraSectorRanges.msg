# Message to publish ranges from a stripe of depth camera data.
# The sector with the minimum range is published as radians 
# left (negative) or right (positive) of image center
# being middle of the image, and -1.0 the left-most sector
float32[] sector_ranges
float32 min_range_rad
float32 min_range