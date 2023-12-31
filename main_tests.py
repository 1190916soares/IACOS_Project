import cv2
import mediapipe as mp
import numpy as np
import math 
import pdb; 

static=False

mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils
pose = mp_pose.Pose()

def draw_landmarks(image, results):
    if results.pose_landmarks:
        mp_draw.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

def calculate_distance(pose_landmarks, point1, point2):
    x1, y1, z1 = pose_landmarks.landmark[point1].x, pose_landmarks.landmark[point1].y, pose_landmarks.landmark[point1].z
    x2, y2, z2 = pose_landmarks.landmark[point2].x, pose_landmarks.landmark[point2].y, pose_landmarks.landmark[point2].z
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

def is_right_hand_raised(pose_landmarks):
    right_shoulder = calculate_distance(pose_landmarks, mp_pose.PoseLandmark.RIGHT_SHOULDER, mp_pose.PoseLandmark.RIGHT_ELBOW)
    right_wrist = calculate_distance(pose_landmarks, mp_pose.PoseLandmark.RIGHT_WRIST, mp_pose.PoseLandmark.RIGHT_ELBOW)

    if right_shoulder > right_wrist:
        return True
    else:
        return False

def test_left(pose_landmarks):
    left_shoulder = pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
    left_elbow = pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW]
    left_wrist = pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST]

    if(left_shoulder.y >= left_wrist.y):
        return True
    else:
        return False


def test_right(pose_landmarks):
    right_shoulder = pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    right_elbow = pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW]
    right_wrist = pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST]
    
    if(right_shoulder.y >= right_wrist.y):
        return True
    else:
        return False

def test_go(pose_landmarks):
    left_shoulder = pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
    left_wrist = pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST]

    right_shoulder = pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    right_wrist = pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST]

    lwrist_pos = [left_wrist.x, left_wrist.y, left_wrist.z]
    rwrist_pos = [right_wrist.x, right_wrist.y, right_wrist.z]
    lshoulder_pos = [left_shoulder.x, left_shoulder.y, left_shoulder.z]
    rshoulder_pos = [right_shoulder.x, right_shoulder.y, right_shoulder.z]

    print("lw", lwrist_pos)
    print("rw",rwrist_pos)
    print("ls",lshoulder_pos)
    print("rs",rshoulder_pos)

    if(is_point_inside_corners(lwrist_pos, rshoulder_pos, lshoulder_pos) and
        is_point_inside_corners(rwrist_pos, rshoulder_pos, lshoulder_pos)):
        return True
    else:
        return False

def is_point_inside_corners(point, corner1, corner2):
    x, y, z = point
    x1, y1, z1 = corner1
    x2, y2, z2 = corner2

    # Check if the point is within the cube's boundaries
    return x1 <= x <= x2 # and z1 <= z <= z2

def test_stop(pose_landmarks):
    left_shoulder = pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
    left_elbow = pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW]
    left_wrist = pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST]

    right_shoulder = pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    right_elbow = pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW]
    right_wrist = pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST]

    angle_left_elbow = calculate_angle(left_wrist, left_elbow, left_shoulder)
    angle_left_shoulder = calculate_angle(left_wrist, left_shoulder, left_elbow)

    angle_right_elbow = calculate_angle(right_wrist, right_elbow, right_shoulder)
    angle_right_shoulder = calculate_angle(right_wrist, right_shoulder, right_elbow)

    #pdb.set_trace()

    if(right_shoulder.y > right_elbow.y and right_elbow > right_wrist.y and 
        angle_right_elbow > 90.0 and angle_right_shoulder < 30.0) or (angle_right_elbow > 110 and angle_right_shoulder < 20):
        return True
    if(left_shoulder.y > left_elbow.y and left_elbow > left_wrist.y and 
        angle_left_elbow > 90.0 and angle_left_shoulder < 30.0) or (angle_left_elbow > 110 and angle_left_shoulder < 20):
        return True
    else:
        return False

def is_left_hand_raised(pose_landmarks):
    left_shoulder = calculate_distance(pose_landmarks, mp_pose.PoseLandmark.LEFT_SHOULDER, mp_pose.PoseLandmark.LEFT_ELBOW)
    left_wrist = calculate_distance(pose_landmarks, mp_pose.PoseLandmark.LEFT_WRIST, mp_pose.PoseLandmark.LEFT_ELBOW)

    if left_shoulder > left_wrist:
        return True
    else:
        return False

def calculate_angle(a, b, c):
    ba = (b.x - a.x, b.y - a.y, b.z - a.z)
    bc = (b.x - c.x, b.y - c.y, b.z - c.z)

    cosine_angle = (ba[0] * bc[0] + ba[1] * bc[1] + ba[2] * bc[2]) / math.sqrt((ba[0] ** 2 + ba[1] ** 2 + ba[2] ** 2) * (bc[0] ** 2 + bc[1] ** 2 + bc[2] ** 2))

    return math.degrees(math.acos(cosine_angle))

def check_hand_stop_raised(wrist, elbow, shoulder):
    # Calculate the angles between the landmarks
    wrist_to_elbow_angle = calculate_angle(wrist, elbow, wrist)
    elbow_to_shoulder_angle = calculate_angle(elbow, shoulder, elbow)

    # Check if the angles are within a suitable range
    is_hand_raised = (90 - wrist_to_elbow_angle < 30) and (90 - elbow_to_shoulder_angle < 30)

    return is_hand_raised

def check_frame(frame,results):
    if results.pose_landmarks:
            if test_go(results.pose_landmarks):
                cv2.putText(frame, 'MOVE FORWARD', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
            elif test_left(results.pose_landmarks):
                cv2.putText(frame, 'LEFT TURN', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
            elif test_right(results.pose_landmarks):
                cv2.putText(frame, 'RIGHT TURN', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
            elif test_stop(results.pose_landmarks):
                cv2.putText(frame, 'STOP SIGN', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)

if(static):
    fwd = cv2.imread('images/FORWARD.jpg')
    fwd_im = cv2.cvtColor(fwd, cv2.COLOR_BGR2RGB)
    fwd_results = pose.process(fwd_im)
    draw_landmarks(fwd, fwd_results)
    check_frame(fwd, fwd_results)
    cv2.imshow('Forward Pose', fwd)

    left = cv2.imread('images/LEFT.jpg')
    left_im = cv2.cvtColor(left, cv2.COLOR_BGR2RGB)
    left_results = pose.process(left_im)
    draw_landmarks(left, left_results)
    check_frame(left, left_results)
    cv2.imshow('LEFT Pose', left)

    right = cv2.imread('images/RIGHT.jpg')
    right_im = cv2.cvtColor(right, cv2.COLOR_BGR2RGB)
    right_results = pose.process(right_im)
    draw_landmarks(right, right_results)
    check_frame(right, right_results)
    cv2.imshow('RIGHT Pose', right)

    stopl = cv2.imread('images/STOP_LEFT.jpg')
    stopl_im = cv2.cvtColor(stopl, cv2.COLOR_BGR2RGB)
    stopl_results = pose.process(stopl_im)
    draw_landmarks(stopl, stopl_results)
    check_frame(stopl, stopl_results)
    cv2.imshow('STOP_LEFT Pose', stopl)

    stopr = cv2.imread('images/STOP_RIGHT.jpg')
    stopr_im = cv2.cvtColor(stopr, cv2.COLOR_BGR2RGB)
    stopr_results = pose.process(stopr_im)
    draw_landmarks(stopr, stopr_results)
    check_frame(stopr, stopr_results)
    cv2.imshow('STOP_RIGHT Pose', stopr)

    stop2 = cv2.imread('images/STOP2.jpg')
    stop2_im = cv2.cvtColor(stop2, cv2.COLOR_BGR2RGB)
    stop2_results = pose.process(stop2_im)
    draw_landmarks(stop2, stop2_results)
    check_frame(stop2, stop2_results)
    cv2.imshow('STOP2 Pose', stop2)

    cv2.waitKey(0)

    cv2.destroyAllWindows()
else:
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(image)
        draw_landmarks(frame, results)
        #pdb.set_trace()
        if results.pose_landmarks:
            if test_go(results.pose_landmarks):
                cv2.putText(frame, 'MOVE FORWARD', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
            elif test_left(results.pose_landmarks):
                cv2.putText(frame, 'LEFT TURN', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
            elif test_right(results.pose_landmarks):
                cv2.putText(frame, 'RIGHT TURN', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
            elif test_stop(results.pose_landmarks):
                cv2.putText(frame, 'STOP SIGN', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)


        cv2.imshow('MediaPipe Pose', frame)

        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()