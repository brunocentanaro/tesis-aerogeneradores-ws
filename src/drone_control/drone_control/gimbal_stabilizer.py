
import asyncio
from mavsdk import System

drone = System()

DELAY_FOR_CORRECTING_GIMBAL = 0.3

# Controls the drone's gimbal to match the drone's attitude
async def manual_control_drone(my_drone):
    while True:
        await asyncio.sleep(DELAY_FOR_CORRECTING_GIMBAL)

        # Fetch the drone's current attitude (roll, pitch, and yaw angles)
        async for attitude in drone.telemetry.attitude_euler():
            roll = attitude.roll_deg
            pitch = attitude.pitch_deg
            # Adjust the gimbal position based on the roll and pitch, keeping yaw constant at -180
            await my_drone.gimbal.set_angles(-roll, -pitch, -180)

async def run_drone():
    # Establish connection to the drone via UDP
    await drone.connect(system_address="udp://:14540")

    # Wait until the drone connects successfully and notify
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    # Checking if Global Position Estimate is ok
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    # Start the control of the drone's gimbal
    asyncio.ensure_future(manual_control_drone(drone))


async def run():
    await asyncio.gather(run_drone())

def main():
    asyncio.ensure_future(run())
    asyncio.get_event_loop().run_forever()
