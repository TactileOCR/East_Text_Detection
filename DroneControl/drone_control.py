import asyncio
import cv2
import pytesseract
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from dotenv import load_dotenv
import shutil

# Load values from the .env file if it exists
load_dotenv()
pytesseract.pytesseract.tesseract_cmd = shutil.which("tesseract") 


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
    upload_result = await drone.mission.upload_mission(mission_plan)
    if upload_result != "Success":
        print(f"Failed to upload mission: {upload_result}")
        return

    # Ensure mission upload was successful
    print("-- Mission uploaded")

    # Start the mission
    print("-- Starting mission")
    start_result = await drone.mission.start_mission()
    if start_result != "Success":
        print(f"Failed to start mission: {start_result}")
        return

async def capture_image(drone):
    # Placeholder for image capture logic
    # In SITL, simulate by loading a test image
    test_image_path = 'test_image.jpg'
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
