#!/usr/bin/env python3
"""
Camera Calibration Script using Checkerboard Pattern

Usage:
1. Print a checkerboard pattern (e.g., 9x6 or 10x7 inner corners)
2. Measure the square size in meters (e.g., 0.025m = 2.5cm)
3. Capture 15-30 images of the checkerboard from different angles/distances
   - Save them in a folder (e.g., calibration_images/)
4. Run this script: python3 calibrate_camera.py
5. Copy the output values to your C++ code

Tips for good calibration:
- Cover all areas of the image (corners, edges, center)
- Tilt the board at various angles
- Vary the distance from camera
- Ensure images are in focus
- Use consistent, good lighting
- Aim for reprojection error < 0.5 pixels
"""

import cv2
import numpy as np
import glob
import os
import sys
import argparse

def calibrate_camera(images_path, checkerboard_size, square_size, show_corners=True):
    """
    Calibrate camera using checkerboard images
    
    Args:
        images_path: Path pattern to calibration images (e.g., 'calibration_images/*.jpeg')
        checkerboard_size: Tuple of (width, height) inner corners (e.g., (9, 6))
        square_size: Size of checkerboard squares in meters (e.g., 0.025)
        show_corners: Whether to display detected corners
    
    Returns:
        cameraMatrix, distCoeffs, mean_error
    """
    
    # Prepare object points (0,0,0), (1,0,0), (2,0,0) ... scaled by square size
    objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    # Arrays to store object points and image points from all images
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane
    
    # Load calibration images
    image_files = glob.glob(images_path)
    
    if len(image_files) == 0:
        print(f"ERROR: No images found at {images_path}")
        print("Please capture calibration images first!")
        sys.exit(1)
    
    print(f"Found {len(image_files)} calibration images")
    print("Processing images...")
    
    successful = 0
    image_size = None
    
    for fname in image_files:
        img = cv2.imread(fname)
        if img is None:
            print(f"Warning: Could not read {fname}")
            continue
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        if image_size is None:
            image_size = gray.shape[::-1]
        
        # Find checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
        
        if ret:
            objpoints.append(objp)
            
            # Refine corner locations for sub-pixel accuracy
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            
            successful += 1
            print(f"✓ {os.path.basename(fname)} - Corners detected")
            
            # Draw and display the corners (skip if no display available)
            if show_corners:
                try:
                    cv2.drawChessboardCorners(img, checkerboard_size, corners2, ret)
                    cv2.imshow('Detected Corners (press any key)', img)
                    cv2.waitKey(300)
                except cv2.error:
                    # No display available (headless environment), skip visualization
                    if successful == 1:
                        print("Note: No display available, skipping visualization")
                    show_corners = False
        else:
            print(f"✗ {os.path.basename(fname)} - No corners found")
    
    if show_corners:
        try:
            cv2.destroyAllWindows()
        except:
            pass
    
    if successful < 10:
        print(f"\nWARNING: Only {successful} images with detected corners.")
        print("You should have at least 10-15 good images for reliable calibration.")
        if successful < 3:
            print("ERROR: Not enough images for calibration!")
            sys.exit(1)
    
    print(f"\n{successful}/{len(image_files)} images successfully processed")
    print("Calibrating camera...")
    
    # Calibrate camera
    ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None)
    
    if not ret:
        print("ERROR: Calibration failed!")
        sys.exit(1)
    
    # Calculate reprojection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], 
                                          cameraMatrix, distCoeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    
    mean_error = mean_error / len(objpoints)
    
    return cameraMatrix, distCoeffs, mean_error


def print_results(cameraMatrix, distCoeffs, mean_error):
    """Print calibration results in C++ format"""
    
    print("\n" + "="*70)
    print("CALIBRATION RESULTS")
    print("="*70)
    
    print("\n📊 Reprojection Error: {:.4f} pixels".format(mean_error))
    if mean_error < 0.3:
        print("   ✓ Excellent calibration!")
    elif mean_error < 0.5:
        print("   ✓ Good calibration")
    elif mean_error < 1.0:
        print("   ⚠ Fair calibration - consider recalibrating with more/better images")
    else:
        print("   ✗ Poor calibration - recalibrate with better images!")
    
    print("\n" + "-"*70)
    print("C++ Code (copy to your localization_node.cpp):")
    print("-"*70)
    
    print("\nstatic const Mat cameraMatrix = (Mat_<float>(3,3) <<")
    print(f"    {cameraMatrix[0,0]:.8e}f, 0.f, {cameraMatrix[0,2]:.8e}f,")
    print(f"    0.f, {cameraMatrix[1,1]:.8e}f, {cameraMatrix[1,2]:.8e}f,")
    print(f"    0.f, 0.f, 1.f);")
    
    print("\nstatic const Mat distCoeffs = (Mat_<float>(1,5) <<")
    print(f"    {distCoeffs[0,0]:.8e}f, {distCoeffs[0,1]:.8e}f, "
          f"{distCoeffs[0,2]:.8e}f, {distCoeffs[0,3]:.8e}f, {distCoeffs[0,4]:.8e}f);")
    
    print("\n" + "-"*70)
    print("Individual Parameters:")
    print("-"*70)
    print(f"Focal Length X (fx): {cameraMatrix[0,0]:.2f} pixels")
    print(f"Focal Length Y (fy): {cameraMatrix[1,1]:.2f} pixels")
    print(f"Principal Point X (cx): {cameraMatrix[0,2]:.2f} pixels")
    print(f"Principal Point Y (cy): {cameraMatrix[1,2]:.2f} pixels")
    print(f"\nDistortion Coefficients:")
    print(f"  k1 (radial): {distCoeffs[0,0]:.6f}")
    print(f"  k2 (radial): {distCoeffs[0,1]:.6f}")
    print(f"  p1 (tangential): {distCoeffs[0,2]:.6f}")
    print(f"  p2 (tangential): {distCoeffs[0,3]:.6f}")
    print(f"  k3 (radial): {distCoeffs[0,4]:.6f}")
    print("="*70 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description='Calibrate camera using checkerboard pattern',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage with default 9x6 checkerboard
  python3 calibrate_camera.py

  # Specify custom checkerboard size
  python3 calibrate_camera.py --width 10 --height 7

  # Use different square size (in meters)
  python3 calibrate_camera.py --square-size 0.03

  # Specify custom images path
  python3 calibrate_camera.py --images "my_calibration/*.png"

  # Don't show corner visualization
  python3 calibrate_camera.py --no-show
        """)
    
    parser.add_argument('--images', type=str, 
                       default='calibration_images/*.jpeg',
                       help='Path pattern to calibration images (default: calibration_images/*.jpeg)')
    parser.add_argument('--width', type=int, default=9,
                       help='Checkerboard width (inner corners) (default: 9)')
    parser.add_argument('--height', type=int, default=6,
                       help='Checkerboard height (inner corners) (default: 6)')
    parser.add_argument('--square-size', type=float, default=0.025,
                       help='Size of checkerboard squares in meters (default: 0.025)')
    parser.add_argument('--no-show', action='store_true',
                       help='Do not display detected corners')
    
    args = parser.parse_args()
    
    print("="*70)
    print("CAMERA CALIBRATION TOOL")
    print("="*70)
    print(f"Checkerboard size: {args.width}x{args.height} inner corners")
    print(f"Square size: {args.square_size} meters")
    print(f"Images pattern: {args.images}")
    print("="*70 + "\n")
    
    # Run calibration
    cameraMatrix, distCoeffs, mean_error = calibrate_camera(
        args.images,
        (args.width, args.height),
        args.square_size,
        show_corners=not args.no_show
    )
    
    # Print results
    print_results(cameraMatrix, distCoeffs, mean_error)
    
    # Save to file
    output_file = "camera_calibration.npz"
    np.savez(output_file, 
             cameraMatrix=cameraMatrix, 
             distCoeffs=distCoeffs,
             error=mean_error)
    print(f"Calibration data saved to: {output_file}")
    print("You can load this in Python with: np.load('camera_calibration.npz')\n")


if __name__ == "__main__":
    main()
