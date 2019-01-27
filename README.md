Flying pathfinding prototype I did way back in 2016 in Unity 5.4 using an Octree

# Screenshots

![Screenshot](https://github.com/simeonradivoev/Flying-Pathfinding/raw/master/Screenshots/Screenshot.png)

# Using in your project

  1. Checkout the source from GitHub
  2. Open in Unity
  3. Run from Unity with `Tools -> Build -> Build Core Package`
  4. Open/create your own unity project
  5. Import the package built in step 3 (it's in the parent directory of the Flying Pathfinding project directory)
  6. Add the `Octree` prefab to your scene. Ensure that it adequately covers the area you want your flying object to fly within.
  7. Add the `RobotMovementController` to any object whose movement should be managed by the Flying Pathfinding algorithm (NOTE: objects to be controllers must have a Rigidbody and a collider)
  9. Add the `RobotRotationController` to any object who should look towards the player after arrival.
