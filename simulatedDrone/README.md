# simulatedDrone
How to playing simulated drone

- Step 1

  ```
  dronekit-sitl copter --home=-7.833720493928131,110.38309035134006,0,0
  ```
  
  `--home=x,y,0,0` for set home location
  
  `--speedup 5` for set speed throtle to 5, default speed is 1
  

- Step 2

  On Windows

  ```
  MAVProxy --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out 127.0.0.1:14552
  ```
  
  On Linux
  
  ```
  mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out 127.0.0.1:14552
  ```
  
  And you will see Mission Planner automatically connects to drone simulation.
  
  ![Screenshot (331)](https://user-images.githubusercontent.com/99522867/157839813-b3aafee6-134a-4289-8f95-f01d42cf4c26.png)
  
  
- Step 3
  
  Simulation without python script.
  
  First open `Actions` tab.
  
  ![Screenshot from 2022-03-23 20-43-33](https://user-images.githubusercontent.com/99522867/159713637-5348b25f-24e9-4b46-842e-ce49c56b5e59.png)

  And then Set Mode to `Stabilize`.
  
  ![Screenshot from 2022-03-23 20-47-11](https://user-images.githubusercontent.com/99522867/159714362-25c75b76-59c4-4d8b-bce8-59018eece1a7.png)

  Click on `Arm/Disarm` button.
  
  ![Screenshot from 2022-03-23 20-51-38](https://user-images.githubusercontent.com/99522867/159715310-90918e39-be70-4dc0-9936-56e2d47fe3b3.png)

  And you will see drone in armed position.
  
  ![Screenshot from 2022-03-23 20-54-45](https://user-images.githubusercontent.com/99522867/159715929-40a81bd8-fbac-4df8-b96c-7c8776276862.png)

  Right click on the map, and select `TakeOff`.
  
  ![Screenshot from 2022-03-23 20-56-42](https://user-images.githubusercontent.com/99522867/159716518-74b83fd3-fde2-4099-b99e-3098d4fe57c2.png)

  And enter the flying altitude of the drone, here I enter the number 5.
  
  ![Screenshot from 2022-03-23 21-00-55](https://user-images.githubusercontent.com/99522867/159717099-ae8dd0db-72b8-4738-8c86-eff0214df5f5.png)
  
# Preview

https://user-images.githubusercontent.com/99522867/161211821-647d884e-5ee4-4bc5-963e-c1ffd446ef63.mp4


# simulatedDrone on Gazebo
