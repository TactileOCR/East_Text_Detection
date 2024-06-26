import asyncio
import cv2
import pytesseract
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

# Initial position
latitude_deg_default = 47.398170327054473
longitude_deg_default = 8.5456490218639658


# Change between taking pics
delta_lat_deg = -7.101107064499956e-05
delta_long_deg = 0.00011301040652256233

async def setup_drone():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
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

def make_mission_item(latitude_deg = latitude_deg_default, longitude_deg = longitude_deg_default):
    item = MissionItem(
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
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
    return item 



async def set_mission(drone):
    mission_items = [
        make_mission_item(),
        make_mission_item(latitude_deg_default + delta_lat_deg, 
                          longitude_deg_default + delta_long_deg)
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

async def capture_image():
    cap = cv2.VideoCapture(0)  # 0 is the default camera

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return None

    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read frame.")
        return None

    # Save the captured frame to a file
    image_path = 'captured_image.png'
    cv2.imwrite(image_path, frame)
    cap.release()

    return image_path

def ocr_from_image(image_path):
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    text = pytesseract.image_to_string(gray)
    print(f"OCR Result: {text}")

async def take_pictures():
    for _ in range(10):
        image_path = await capture_image()
        if image_path:
            ocr_from_image(image_path)
        asyncio.sleep(5)


async def main():
    await take_pictures()
    drone = await setup_drone()
    await arm_and_takeoff(drone)
    await set_mission(drone)
    await asyncio.sleep(20)  # Wait for mission to complete




if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
