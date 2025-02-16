import time
from quadruped_controller_binding import QuadrupedController

def test_quadruped_controller():
    # QuadrupedController 객체 생성
    controller = QuadrupedController()

    # 1. 조인트 이름 가져오기
    joint_names = controller.getJointNames()
    print("Joint Names:", joint_names)

    start = time.time()
    while time.time() - start:
        time.sleep(0.1)
        print(controller.getJointPositions()[0])


if __name__ == "__main__":
    test_quadruped_controller()