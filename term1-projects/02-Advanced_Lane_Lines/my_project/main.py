#imports
from LaneLinesDetection import LaneLinesDetection
import cv2

def main():

    display = True
    images_for_calibration_path = './camera_cal/calibration*.jpg'

    LLD = LaneLinesDetection(display=display)

    test_image_path = './test_images/straight_lines1.jpg'
    test_image = cv2.imread(test_image_path)

    if display:
        cv2.imshow('Original image', test_image)
        cv2.waitKey(50)
        cv2.imwrite('./output_images/00-original_image.jpg',test_image)
        cv2.destroyAllWindows()

    LLD.calibrate_camera(images_for_calibration_path)
    LLD.process_image(test_image)

    print("I'm DONE!")

if __name__ == "__main__":
    main()