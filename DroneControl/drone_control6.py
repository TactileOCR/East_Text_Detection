#----------------------------------------------------------------
# File:     drone_control6.py
#----------------------------------------------------------------
#
# Author:   Marek Rychlik (rychlik@arizona.edu)
# Date:     Thu Jun 27 16:42:51 2024
# Copying:  (C) Marek Rychlik, 2020. All rights reserved.
# 
#----------------------------------------------------------------
# In this version we change a running mission
import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan, MissionError

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

async def set_initial_mission(drone):
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
    try:
        await drone.mission.upload_mission(mission_plan)
        print("-- Mission uploaded")
    except MissionError as e:
        print(f"Mission upload failed: {e}")
        return

    # Set to return to launch after mission is complete
    await drone.mission.set_return_to_launch_after_mission(True)

    # Start the mission
    print("-- Starting mission")
    try:
        await drone.mission.start_mission()
        print("-- Mission started")
    except MissionError as e:
        print(f"Mission start failed: {e}")

async def modify_mission(drone):
    print("-- Downloading current mission")
    try:
        current_mission_plan = await drone.mission.download_mission()
        current_mission_items = current_mission_plan.mission_items
    except MissionError as e:
        print(f"Mission download failed: {e}")
        return

    # Modify the mission items as needed
    # Example: Remove the second mission item and modify the first one
    if len(current_mission_items) > 1:
        print("-- Removing the second mission item")
        current_mission_items.pop(1)
    
    if len(current_mission_items) > 0:
        print("-- Modifying the first mission item")
        current_mission_items[0].relative_altitude_m = 20  # Change altitude
        current_mission_items[0].longitude_deg = 8.545  # Change longitude

    # Create a new mission plan with the modified items
    updated_mission_plan = MissionPlan(current_mission_items)

    # Upload the updated mission plan
    print("-- Uploading updated mission plan")
    try:
        await drone.mission.upload_mission(updated_mission_plan)
        print("-- Updated mission plan uploaded")
    except MissionError as e:
        print(f"Updated mission upload failed: {e}")

    print("-- Resuming mission")
    try:
        await drone.mission.start_mission()
        print("-- Mission resumed")
    except MissionError as e:
        print(f"Resuming mission failed: {e}")

async def main():
    drone = await setup_drone()
    await arm_and_takeoff(drone)
    await set_initial_mission(drone)

    # Simulate waiting for some time during the mission
    await asyncio.sleep(20)  # Wait for the mission to partially complete

    # Modify the mission plan
    await modify_mission(drone)

    await asyncio.sleep(20)  # Wait for the updated mission to complete

    # Return to Launch (RTL) after completing the mission
    print("-- Returning to Launch")
    try:
        await drone.action.return_to_launch()
        print("-- RTL initiated")
    except MissionError as e:
        print(f"RTL failed: {e}")

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
