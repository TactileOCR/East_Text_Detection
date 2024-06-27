#----------------------------------------------------------------
# File:     drone_control4.py
#----------------------------------------------------------------
#
# Author:   Marek Rychlik (rychlik@arizona.edu)
# Date:     Thu Jun 27 16:54:40 2024
# Copying:  (C) Marek Rychlik, 2020. All rights reserved.
# 
#----------------------------------------------------------------
# In this version we fly a mission and asynchronously
# take pictures

import asyncio
import cv2
import pytesseract
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

async def setup_drone():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered")
            break

    return drone

async def arm_and_takeoff(drone):
    print("-- Arming")
    await drone.action.arm()
    await asyncio.sleep(1)

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(10)

async def set_mission(drone):
    mission_items = [
        MissionItem(
            latitude_deg=47.398170327054473,
            longitude_deg=8.5456490218639658,
            relative_altitude_m=10,
            speed_m_s=10,
            is_fly_through=True,
            gimbal_pitch_deg=float('nan'),
            gimbal_yaw_deg=float('nan'),
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=float('nan'),
            camera_photo_interval_s=float('nan'),
            acceptance_radius_m=1.0,
            yaw_deg=float('nan'),
            camera_photo_distance_m=float('nan')
        ),
        MissionItem(
            latitude_deg=47.398241338125118,
            longitude_deg=8.5455360114574432,
            relative_altitude_m=10,
            speed_m_s=10,
            is_fly_through=True,
            gimbal_pitch_deg=float('nan'),
            gimbal_yaw_deg=float('nan'),
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=float('nan'),
            camera_photo_interval_s=float('nan'),
            acceptance_radius_m=1.0,
            yaw_deg=float('nan'),
            camera_photo_distance_m=float('nan')
        )
    ]
    
    mission_plan = MissionPlan(mission_items)

    # Upload the mission
    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)
    print("-- Mission uploaded")

    # Start the mission
    print("-- Starting mission")
    await drone.mission.start_mission()
    print("-- Mission started")

async def update_mission(drone):
    print("-- Pausing current mission")
    await drone.mission.pause_mission()
    await asyncio.sleep(5)

    print("-- Updating mission plan")
    additional_items = [
        MissionItem(
            latitude_deg=47.398348586,
            longitude_deg=8.545404493,
            relative_altitude_m=10,
            speed_m_s=10,
            is_fly_through=True,
            gimbal_pitch_deg=float('nan'),
            gimbal_yaw_deg=float('nan'),
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=float('nan'),
            camera_photo_interval_s=float('nan'),
            acceptance_radius_m=1.0,
            yaw_deg=float('nan'),
            camera_photo_distance_m=float('nan')
        )
    ]

    # Fetch current mission items
    current_mission_plan = await drone.mission.download_mission()
    current_mission_items = current_mission_plan.mission_items

    # Combine current mission items with additional items
    updated_mission_items = current_mission_items + additional_items
    updated_mission_plan = MissionPlan(updated_mission_items)

    print("-- Uploading updated mission plan")
    await drone.mission.upload_mission(updated_mission_plan)
    print("-- Updated mission plan uploaded")

    print("-- Resuming mission")
    await drone.mission.start_mission()

async def capture_image(counter):
    cap = cv2.VideoCapture(0)  # 0 is the default camera

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return None

    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read frame.")
        return None

    # Save the captured frame to a file
    image_path = f'captured_image_{counter}.png'
    cv2.imwrite(image_path, frame)
    cap.release()

    return image_path

def ocr_from_image(image_path):
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    text = pytesseract.image_to_string(gray)
    print(f"OCR Result: {text}")

async def main():
    drone = await setup_drone()
    await arm_and_takeoff(drone)
    await set_mission(drone)

    image_capture_task = asyncio.create_task(capture_images_during_mission())

    await asyncio.sleep(20)  # Wait for mission to complete or partially complete

    # Update the mission plan with new items
    await update_mission(drone)

    await asyncio.sleep(20)  # Wait for updated mission to complete

    image_capture_task.cancel()  # Stop image capture after the mission

async def capture_images_during_mission():
    counter = 0
    while True:
        image_path = await capture_image(counter)
        if image_path:
            ocr_from_image(image_path)
        counter += 1
        await asyncio.sleep(5)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
