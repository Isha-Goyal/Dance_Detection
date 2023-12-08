"""
Uses pre-trained yolo model to determine pose in a video.

input: video_path
note: showing annotated_frame will show the default visual that comes with the model (with all the colors and lines).
    Showing plotted_img will show the visual we are using for info gathering.
"""

import cv2
from ultralytics import YOLO

def find_keypoints(model, cap):

    ret = []

    # Loop through the video frames
    while cap.isOpened():
    #     # Read a frame from the video
        success, frame = cap.read()

        if success:
            # Run YOLOv8 inference on the frame
            results = model(frame)

            # Visualize the results on the frame
            annotated_frame = results[0].plot()

            # plot by ourselves with keypoint info
            keypoints = results[0].keypoints.xy.cpu().numpy()
            ret.append(keypoints)

            plotted_img = frame

            for i in range(len(keypoints[0])):
                pt = keypoints[0][i]
                ctr = (int(pt[0]), int(pt[1]))
                print(pt)
                plotted_img = cv2.circle(plotted_img, ctr, radius=2, color=(0, 0, 255), thickness=-1)
                plotted_img = cv2.putText(plotted_img, str(i),ctr, fontFace=0, fontScale=1, color=(0, 0, 255), thickness=2)

            # Display
            cv2.imshow("YOLOv8 Inference", plotted_img)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            # Break the loop if the end of the video is reached
            break

    # Release the video capture object and close the display window
    cap.release()
    cv2.destroyAllWindows()

    return ret

def extract_mvmt(keypoints):
    """
    Takes keypoints and returns it in some format that can be used to control the neatos.
    """

    set = [0, 10, 9, 16, 15] # which keypoints you want to pull out
    cleaned = []

    # Create an array with only the keypoints we want
    for i in range(len(keypoints)):
        frame = []
        for j in set:
            frame.append(keypoints[i][0][j])
        cleaned.append(frame)

    return cleaned # this should be of size # of frames and the array for each frame should have the number of arrays as we chose keypoints in the set

def main():
    # Load the YOLOv8 model
    model = YOLO('yolov8m-pose.pt')

    
    cap = cv2.VideoCapture(0)
    # cv2.imwrite('img2.png', frame)
    # cv2.imshow("img1", frame)

    kpts = find_keypoints(model, cap)

    cleaned = extract_mvmt(kpts)
    print(cleaned)

if __name__ == "__main__":
    main()