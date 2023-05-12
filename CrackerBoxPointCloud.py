# %%
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.perception import Concatenate
from pydrake.systems.framework import DiagramBuilder

from manipulation.scenarios import AddRgbdSensors
from manipulation.utils import FindResource, AddPackagePaths

import matplotlib.pyplot as plt
import mpld3
import numpy as np
import pydot
import CrackerBoxPointCloud
from IPython.display import SVG, clear_output, display
from pydrake.all import (AddMultibodyPlantSceneGraph, AngleAxis,
                         DiagramBuilder, FindResourceOrThrow, Integrator,
                         JacobianWrtVariable, LeafSystem, MathematicalProgram,
                         MeshcatVisualizer, SnoptSolver, Solve, eq, ge, le,
                         MultibodyPlant, MultibodyPositionToGeometryPose,
                         Parser, PiecewisePolynomial, PiecewisePose,
                         Quaternion, Rgba, RigidTransform, RotationMatrix,
                         SceneGraph, Simulator, StartMeshcat, TrajectorySource)

from manipulation import running_as_notebook, FindResource
from manipulation.scenarios import (AddIiwaDifferentialIK,
                                    MakeManipulationStation)
from manipulation.meshcat_utils import AddMeshcatTriad
from pydrake.manipulation.planner import (
    DifferentialInverseKinematicsIntegrator,
    DifferentialInverseKinematicsParameters,
    DifferentialInverseKinematicsStatus)
from pydrake.manipulation.planner import DoDifferentialInverseKinematics

if running_as_notebook:
    mpld3.enable_notebook()
    
# asds






# %%

model_directives = """
directives:
- add_frame:
    name: cracker_box_origin
    X_PF:
        base_frame: world
        rotation: !Rpy { deg: [-90.0, 0.0, -90.0 ]}
        translation: [0, 0, 0.09515]

- add_model:
    name: cracker_box
    file: package://drake/manipulation/models/ycb/sdf/003_cracker_box.sdf

- add_weld:
    parent: cracker_box_origin
    child: cracker_box::base_link_cracker


- add_frame:
    name: camera0_staging
    X_PF:
        base_frame: world
        rotation: !Rpy { deg: [0, 0, 15.0]}

- add_frame:
    name: camera1_staging
    X_PF:
        base_frame: world
        rotation: !Rpy { deg: [0, 0, 130.0]}

- add_frame:
    name: camera2_staging
    X_PF:
        base_frame: world
        rotation: !Rpy { deg: [0, 0, 245.0]}

- add_frame:
    name: camera0_origin
    X_PF:
        base_frame: camera0_staging
        rotation: !Rpy { deg: [-100.0, 0, 90.0]}
        translation: [.5, 0, .2]

- add_model:
    name: camera0
    file: package://manipulation/camera_box.sdf

- add_weld:
    parent: camera0_origin
    child: camera0::base

- add_frame:
    name: camera1_origin
    X_PF:
        base_frame: camera1_staging
        rotation: !Rpy { deg: [-100.0, 0, 90.0]}
        translation: [.5, 0, .2]

- add_model:
    name: camera1
    file: package://manipulation/camera_box.sdf

- add_weld:
    parent: camera1_origin
    child: camera1::base

- add_frame:
    name: camera2_origin
    X_PF:
        base_frame: camera2_staging
        rotation: !Rpy { deg: [-100.0, 0, 90.0]}
        translation: [.5, 0, .2]

- add_model:
    name: camera2
    file: package://manipulation/camera_box.sdf

- add_weld:
    parent: camera2_origin
    child: camera2::base
""" 

# %%
# builder = DiagramBuilder()
# station = builder.AddSystem(MakeManipulationStation(model_directives=model_directives))


# %%
def CrackerBoxExampleSystem():
    builder = DiagramBuilder()

    # Create the physics engine + scene graph.
    # plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    # parser = Parser(plant)
    # AddPackagePaths(parser)
    # # parser.AddModelFromString(model_directives, 'dmd.yaml')
    # plant.Finalize()
    
    
    scene_graph = builder.AddSystem(SceneGraph())
    plant = builder.AddSystem(MultibodyPlant(time_step=0.0))


    # parser = Parser(plant)
    # AddPackagePaths(parser)
    station = builder.AddSystem(MakeManipulationStation(model_directives=model_directives))
    plant.Finalize()
    

    # plant = station.GetSubsystemByName("plant")  
    # scene_graph = station.GetSubsystemByName("scene_graph")

    
    # Register scene_graph with the plant.
    # plant.RegisterAsSourceForSceneGraph(scene_graph) # geometry source already registered



    # Add a visualizer just to help us see the object.
    use_meshcat = False
    if use_meshcat:
        meshcat = builder.AddSystem(MeshcatVisualizer(scene_graph))
        builder.Connect(scene_graph.get_query_output_port(),
                        meshcat.get_geometry_query_input_port())

    AddRgbdSensors(builder, plant, scene_graph)

    diagram = builder.Build()
    diagram.set_name("depth_camera_demo_system")
    return diagram


def CrackerBoxPointCloud(normals=False, down_sample=True):
    system = CrackerBoxExampleSystem()

    systems = system.GetSystems()
    for system in systems:
        system.get_name()
    # print("get subystem by name plant: ", system.GetSubsystemByName("plant"))
    

    context = system.CreateDefaultContext()
    plant = system.GetSubsystemByName("plant")
    # plant_context = plant.GetMyMutableContextFromRoot(context)
    plant_context = system.GetMutableSubsystemContext(plant, context)

    pcd = []
    for i in range(3):
        cloud = system.GetOutputPort(f"camera{i}_point_cloud").Eval(context)

        # Crop to region of interest.
        pcd.append(cloud.Crop(lower_xyz=[-.3, -.3, -.3], upper_xyz=[.3, .3,
                                                                    .3]))

        if normals:
            pcd[i].EstimateNormals(radius=0.1, num_closest=30)

            camera = plant.GetModelInstanceByName(f"camera{i}")
            body = plant.GetBodyByName("base", camera)
            X_C = plant.EvalBodyPoseInWorld(plant_context, body)
            pcd[i].FlipNormalsTowardPoint(X_C.translation())

    # Merge point clouds.
    merged_pcd = Concatenate(pcd)
    if not down_sample:
        return merged_pcd
    # Down sample.
    down_sampled_pcd = merged_pcd.VoxelizedDownSample(voxel_size=0.005)
    return down_sampled_pcd
