from pxr import UsdPhysics, PhysxSchema, Gf, PhysicsSchemaTools, UsdGeom
import omni

stage = omni.usd.get_context().get_stage()

# Adding a Cube
path = "/World/Cube"
cubeGeom = UsdGeom.Cube.Define(stage, path)
cubePrim = stage.GetPrimAtPath(path)
size = 0.5
offset = Gf.Vec3f(0.5,0.2,1.0)
cubeGeom.CreateSizeAttr(size)
cubeGeom.AddTranslateOp().Set(offset)

# Attach Rigid Body and Collision Preset
rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
rigid_api.CreateRigidBodyEnabledAttr(True)
UsdPhysics.CollisionAPI.Apply(cubePrim)
