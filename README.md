# Overview
This is my Inverse Kinematics project built as an extension to Godot. It allows for a variable
number of joints in a chain of joints for IK. It also allows for the chain
itself to be moved around by defining a marker node. For a walkthrough
on how this project was done, follow this [link](https://jlkump.github.io/projects/inverse-kinematics-godot.html).

## Build the example project
If you would like to see the example project for yourself, follow the steps below:
1. Clone the repo
2. Traverse to the `./godot-cpp directory` and `run git submodule update --init`
3. Checkout the right version for GDExtension using the command `git checkout 4eed2d7be017ef06521665604990114c9fff7c77` (same directory as the previous step)
4. Traverse to `./godot-cpp` and run `scons platform=<your_platform> -j8 custom_api_file=../extension_api.json`
5. Traverse to `./` directory (`cd ..` from godot-cpp)
6. Run `scons platform=<your platform> -j8`
7. Open the `./Project` godot project in Godot (`godot ./Project/godot.project`)
Then you can run the project in editor or build the exectuable for your own platform.

## Use the tool
If you would like to add this extension to your own project, first follow the steps
for setting up GDExtension from Godot at this [link](https://docs.godotengine.org/en/stable/tutorials/scripting/gdextension/gdextension_cpp_example.html).

With that complete, add the following files from my src to your src:
1. helpers.h
2. inverse_kinematic_chain.h/cpp

Then register the InverseKinematicChain class in register_types.cpp. The following is what it should look like:
```
void initialize_project_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
    ...
    ClassDB::register_class<InverseKinematicChain>();
    ...
}
```
Finally, in the terminal perform the command `scons platform=<your_platfom>` from the root of the project to build the
dlls for use in Godot. With that, thet tool should be good to use for your own projects.
