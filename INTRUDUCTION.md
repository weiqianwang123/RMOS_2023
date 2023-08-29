# Introduction for  RMOS


## Preview
RMOS, which stands for Robomaster Open Source, is a software framework developed specifically for 
the DJI RoboMaster competition. It was created by members of the IRobot Vision Team at Xidian
University . Drawing inspiration from existing open-source technologies, 
RMOS has undergone multiple iterations and optimizations over several competition seasons.
It offers a relatively clear and expandable framework along with stable performance.

The primary objective of RMOS is to equip competition robots with the capability of automatic targeting. This capability encompasses various scenarios, including:

-    Stationary targets at varying distances, ranging from close to far.
-    Targets that move relatively uniformly at middle to close distances.
 -   Rotating targets at middle to close distances with relatively uniform angular speeds.
 -   Targets at middle to close distances that exhibit simultaneous uniform linear and angular motion.

RMOS provides a foundation for robots to autonomously achieve precise targeting and engagement in the dynamic and challenging environment of the RoboMaster competition.



## Key points of RMOS(will be extended in the near future)


### 1.  The framework of RMOS (auther:Qianwei Wang)

For the development of this season's automatic targeting system, 
we have chosen to build upon the ROS2 (Robot Operating System 2) platform. 
Based on our experience from previous seasons, we've realized that without a well-defined 
framework, the development of the targeting algorithm can become chaotic during the optimization 
process over the entire competition season. This becomes especially challenging when different 
individuals are responsible for implementing various functionalities, often leading to a situation 
resembling a "pile of code."

Thus, when constructing our automatic targeting framework, our primary goal was to maximize 
extensibility. Leveraging the principles of C++ base classes and inheritance, we have encapsulated 
the majority of external interfaces within the algorithm components. Additionally, we have decoupled 
the algorithmic aspects from ROS2 nodes entirely. Instead, the nodes simply invoke the pre-encapsulated 
common interfaces. This approach serves to standardize the subsequent development process to a 
considerable extent. Developers only need to inherit the external interfaces from the base class 
and proceed with the internal development.

By adopting this approach, we aim to maintain a structured and organized 
development process throughout the evolution of the automatic targeting system, 
ensuring that contributions from different team members can be seamlessly integrated 
without creating a tangled mess of code.

![img.png](pic%2Fimg.png)
![img_1.png](pic%2Fimg_1.png)

### 2. Vehicle State Estimation

During this competition season, we adopted an open-source approach inspired by Chen Jun from South China Normal University for establishing the overall vehicle state. As the referenced approach is open-source, there is no need to elaborate on that part. Instead, I will focus on the enhancements we made to the modeling scheme based on this open-source solution.

In the heat of competition, where split-second decisions are crucial, we often witness intense clashes and rapid movements between vehicles. Confrontations between two vehicles can determine victory or defeat in an instant. This necessitates a rapid convergence of the model from establishment to stabilization. However, this speed requirement also implies that the model is susceptible to errors due to external inaccuracies and could lead to significant fluctuations.

To address this, we recognized that many vehicles on the competition field adhere to certain consistent constraints. For example, the difference in radius between two vehicles generally does not exceed 0.15 meters, and the height difference between adjacent armor panels is typically less than 0.055 meters. Leveraging these inherent constraints, we have introduced limitations to the model. By incorporating these constraints, we can enhance the algorithm's robustness effectively.

This approach not only speeds up the convergence of the model but also helps mitigate errors caused by external factors, leading to more stable and accurate estimations of the vehicle's state. In the high-stress and dynamic environment of the competition, such enhancements contribute to a more reliable and effective overall performance of the vehicle's state estimation.



###  3. Automatic Firing Mechanism

When dealing with rapidly rotating objects or when an opponent hero is approaching a rotating outpost, our gimbal system doesn't always track the target perfectly. In such scenarios, relying on the operator to control the firing can lead to wastage of ammunition. To address this issue, in this competition season, we introduced an automatic firing mechanism across all units. The fundamental principle behind this mechanism is as follows:

Assuming the bullets fired from the barrel travel along an arc, the plane containing this arc should be approximately perpendicular to the plane containing the barrel. We refer to this plane as the "bullet plane," as all our fired bullets lie on this plane. Given that there is no roll angle difference between our camera and the barrel, we can postulate that there's an intersection line between the bullet plane and the camera's imaging plane. Under ideal conditions (perfect alignment between the barrel's center and the camera's optical center), if we use an image size of 1280x720, this intersection line would pass through (640, 360), forming a straight line perpendicular to the upper and lower boundaries of the image. In our camera image, all fired bullets theoretically lie along this line, which we call the "firing line."

With the calculated firing line, we transform the predicted position from the absolute coordinate system to the camera coordinate system, and then further to the pixel coordinate system. At this point, the predicted point becomes a two-dimensional pixel point on the image plane. We determine whether firing should occur by assessing whether the distance between this point and the "firing line" falls below a predefined threshold.

By implementing this automatic firing mechanism, we can optimize our fire control strategy, ensuring that shots are taken efficiently even when the tracking accuracy is compromised due to high-speed rotations or dynamic conditions.



![img_2.png](pic%2Fimg_2.png)

Simultaneously, the firing threshold isn't a static value. Considering the efficiency of engagement, we've made the firing threshold variable inversely proportional to the rotational speed of the target object. When the target object is rotating at low speeds or not rotating at all, and only experiencing translation or remaining stationary, the threshold is set to a high value. This essentially indirectly deactivates the automatic firing, transferring the firing authority back to the operator. Only when the target vehicle is rotating at high speeds will controlled firing occur in a regulated manner.

This approach ensures that shots are taken more
judiciously. 
It prevents unnecessary firing when the target is relatively slow-moving or not rotating, preserving 
ammunition. However, when confronting a rapidly rotating object, the threshold adapts to allow for 
controlled firing to improve the chances of a successful hit. The same parameter settings are applied 
when engaging a hero near a rotating outpost, as the scenario shares similarities with engaging rapidly
rotating objects.

![img_3.png](pic%2Fimg_3.png)

ChatGPT

The approaches mentioned above assumed a roll angle of 0 for the gimbal. However, to accommodate a broader range of scenarios on the field, such as when a hero needs to engage a rotating outpost from a side position, we have taken steps to enhance our system. Specifically, we read the quaternion values from the gimbal's gyroscope to determine the roll angle of the gimbal. We then add this roll angle to the firing line's calculation.

By incorporating the gimbal's actual roll angle, we are better equipped to adapt to various battlefield situations, where the orientation of the vehicle might not align with the standard assumptions. This enhancement allows us to maintain accuracy and effectiveness in targeting, regardless of the gimbal's roll orientation.


![img_4.png](pic%2Fimg_4.png)