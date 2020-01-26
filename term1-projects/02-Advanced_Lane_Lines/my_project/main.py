#imports
from LaneLinesDetection import LaneLinesDetection
import cv2
import argparse

def main():
    parser = argparse.ArgumentParser(description='Process a video/image, highlighting lane and computing curve radius.')
    parser.add_argument('mode', type=str, help='mode: {image, video}')
    
    args = parser.parse_args()

    if args.mode == 'video':
        video()
    elif args.mode == 'image':
        image()
    else:
        print('ERROR: Not a recognized mode!')

def image():
    print('Image Mode!\n')

    display = True
    LLD = LaneLinesDetection(display=display)
    images_for_calibration_path = './camera_cal/calibration*.jpg'
    LLD.calibrate_camera(images_for_calibration_path)

    test_image_path = './test_images/test4.jpg'
    test_image = cv2.imread(test_image_path)

    if display:
        cv2.imshow('Original image', test_image)
        cv2.waitKey(10)
        cv2.imwrite('./output_images/00-original_image.jpg',test_image)
        cv2.destroyAllWindows()

    LLD.process_image(test_image)

    print("I'm DONE!")

def video():
    from moviepy.editor import VideoFileClip
    print('Video Mode!\n')

    display = False
    LLD = LaneLinesDetection(display=display)
    images_for_calibration_path = './camera_cal/calibration*.jpg'
    LLD.calibrate_camera(images_for_calibration_path)    

    filename = 'project_video.mp4'
    clip = VideoFileClip('./videos/'+filename)
    out= clip.fl_image(LLD.process_image)
    out.write_videofile('videos/processed_'+filename, audio=False, verbose=False)
    print("I'm DONE!")

if __name__ == "__main__":
    main()