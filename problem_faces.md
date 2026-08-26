# Problems Faced & Solutions

## 1. TF Extrapolation Error into the Future
**Problem:** The perception pipeline experienced TF extrapolation errors (looking into the future) when transforming camera data.
**Solution:** Solved by strictly synchronizing the `rgb`, `depth`, and `odom` topics using `message_filters.ApproximateTimeSynchronizer`. By directly injecting the synchronized odometry pose into the callback, the need for asynchronous TF lookups was eliminated, completely resolving the extrapolation mismatch.

## 2. Map Over-Segmentation (Duplicate Objects)
**Problem:** The tracker spawns multiple versions of the same physical object (e.g., 14 chairs when there are only 2). This happens because as the robot moves, the visible 3D centroid shifts >0.5m, and the visual CLIP feature changes drastically from the first frame. Additionally, YOLO outputs low-confidence "ghost" detections (like false persons).
**Solution:** 
1. **Feature Averaging:** Update `LocalObject` to calculate a cumulative moving average of the CLIP feature so it smoothly morphs as the perspective changes.
2. **YOLO Confidence Threshold:** Extracted the threshold used by DualMap (`fastsam_confidence: 0.80`) from their config and applied it directly to our YOLO detections. This strict `conf >= 0.80` filter immediately eliminates hallucinated low-confidence objects from ever entering the map.
3. **Map-Level Merging:** Implement a periodic sweep in `MapManager` that merges objects if they share significant 3D overlap, mimicking DualMap's self-matching approach.
