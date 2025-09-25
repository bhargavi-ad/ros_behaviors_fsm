Drive in a Triangle:

Concept:
The draw_triangle node has the Neato draw one equaliteral triangle before stopping. Throughout the entire time the Neato is drawing a triangle, if the Neato bumps into any object and activates one of its four bump sensors, the Neato will automatically stop driving. 

Structure:

This node is multi-threaded. This means that it is constantly checking whether the bump sensor has been activated as it runs the "run triangle" loop. This multi-thread structure allows for both processes happen simultaneously. The benefit of this is that the neato can and will be interrupted at any instance of the neato running into an object, preventing the motors from inadvertantly stalling.

This node publishes the neato wheel velocity to the topic "cmd_vel". A new value is published every time the Neato drives straight, turns, or stops. This node is also subscribed to the topic "Bump". A callback function runs each time the value published to topic "Bump" changes. A value of 0 from the sensor means that no object is touching the bump sensor and 1 means that an object has activated one of the four sensors located on the neato. When the callback function runs, the value of the variable controlling the state of the bump sensor changes. This variable can then be used to determine whether the loop running the Neato must stop. 

Limitation:
One limitation of this method is that the movement of the Neato is dependent on the internal, ideal clock of the Neato. In order to turn 120 degrees, the current structure takes into account the angular velocity of the neato and sets a sleep for the appropriate amoutn of time until the Neato would have completed 120 degrees. The same method is used to have the Neato drive a certain distance based on its linear velocity. However, a major drawback of this is that the Neato doesn't move the same in real life as it does in simulation. Factors such as friction and the robots inertia change the speed at which the Neato can stop, turn, and drive. This means that the timing is not entirely accurate and the angle turns or distance driving in real life is not representative of the exact value determined via simulation