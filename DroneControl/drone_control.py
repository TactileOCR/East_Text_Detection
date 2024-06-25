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
            if hasattr(state,'uuid'):
                print(f"Drone discovered with UUID: {state.uuid}")
            else:
                print(f"Drone discovered with no UUID")
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
    mission_items = []
    mission_items.append(MissionItem(
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
        acceptance_radius_m=1.0,  # You need to provide these additional parameters
        yaw_deg=float('nan'),
        camera_photo_distance_m=float('nan')
    ))
    mission_items.append(MissionItem(
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
        acceptance_radius_m=1.0,  # You need to provide these additional parameters
        yaw_deg=float('nan'),
        camera_photo_distance_m=float('nan')
    ))
    
    mission_plan = MissionPlan(mission_items)
    await drone.mission.set_return_to_launch_after_mission(True)
    await drone.mission.upload_mission(mission_plan)

    print("-- Starting mission")
    await drone.mission.start_mission()

async def capture_image(drone):
    # Placeholder for image capture logic
    # In SITL, simulate by loading a test image
    test_image_path = 'test_image.png'
    ocr_from_image(test_image_path)

def ocr_from_image(image_path):
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    text = pytesseract.image_to_string(gray)
    print(f"OCR Result: {text}")

async def main():
    drone = await setup_drone()
    await arm_and_takeoff(drone)
    await set_mission(drone)
    await asyncio.sleep(20)  # Wait for mission to complete
    await capture_image(drone)
    
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())


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
            if hasattr(state,'uuid'):
                print(f"Drone discovered with UUID: {state.uuid}")
            else:
                print(f"Drone discovered with no UUID")
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
    mission_items = []
    mission_items.append(MissionItem(47.398170327054473, 8.5456490218639658, 10, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan')))
    mission_items.append(MissionItem(47.398241338125118, 8.5455360114574432, 10, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan')))
    
    mission_plan = MissionPlan(mission_items)
    await drone.mission.set_return_to_launch_after_mission(True)
    await drone.mission.upload_mission(mission_plan)

    print("-- Starting mission")
    await drone.mission.start_mission()

async def capture_image(drone):
    # Placeholder for image capture logic
    # In SITL, simulate by loading a test image
    test_image_path = 'test_image.png'
    ocr_from_image(test_image_path)

def ocr_from_image(image_path):
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    text = pytesseract.image_to_string(gray)
    print(f"OCR Result: {text}")

async def main():
    drone = await setup_drone()
    await arm_and_takeoff(drone)
    await set_mission(drone)
    await asyncio.sleep(20)  # Wait for mission to complete
    await capture_image(drone)
    
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
