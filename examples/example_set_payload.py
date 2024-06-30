"""
Created on 2024/6/30 
Author: Hao Chen (chen960216@gmail.com)
"""
if __name__ == "__main__":
    from gofa_con import GoFaArmController

    # initialize the GoFaArm Controller
    arm = GoFaArmController(toggle_debug=False)
    arm.set_payload(mass=1.2)
    arm.stop()
