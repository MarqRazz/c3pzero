import carb
from omni.kit.scripting import BehaviorScript
import carb.events
import omni.timeline
import omni.isaac.core.utils.rotations as rotations_utils
from pxr import UsdGeom, Gf
import random

class C3PzeroContainer(BehaviorScript):
    def on_init(self):
        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")
        self._prim = self.stage.GetPrimAtPath(self.prim_path)
        timeline_stream = self.timeline.get_timeline_event_stream()
        self._timeline_sub = timeline_stream.create_subscription_to_pop(self._on_timeline_event)
        self._playing = self.timeline.is_playing()
        self._rotate = 0.0
        color_r = random.uniform(0,1)
        color_g = random.uniform(0,1)
        color_b = random.uniform(0,1)

        # value = prims_utils.get_prim_parent(prim=prim)  # pxr.Usd.Prim
        # value = prims_utils.get_prim_children(prim=prim)  # pxr.Usd.Prim
        # value = prims_utils.create_prim(prim_path=prim_path,  # str
        #                                 prim_type="Xform",  # str
        #                                 position=None,  # typing.Union[typing.Sequence[float], NoneType]
        #                                 translation=None,  # typing.Union[typing.Sequence[float], NoneType]
        #                                 orientation=None,  # typing.Union[typing.Sequence[float], NoneType]
        #                                 scale=None,  # typing.Union[typing.Sequence[float], NoneType]
        #                                 usd_path=None,  # typing.Union[str, NoneType]
        #                                 semantic_label=None,  # typing.Union[str, NoneType]
        #                                 semantic_type="class",  # str
        #                                 attributes=None)  # typing.Union[dict, NoneType]
        
        # physics_utils.set_rigid_body_enabled(_value=_value,
        #                                      prim_path=prim_path)
        # value = prims_utils.get_prim_property(prim_path=prim_path,  # str
        #                                       property_name=property_name)  # str
        # value = semantics_utils.get_semantics(prim=prim)  # pxr.Usd.Prim

        self._prim.GetAttribute("primvars:displayColor").Set(
            [Gf.Vec3f(color_r, color_g, color_b)]
        )
        # self._prim.GetAttribute("primvars:displayOpacity").Set(
        #     [Gf.Vec3f(color_r, color_g, color_b)]
        # )
        # xf = UsdGeom.Xformable(self._prim)
        # xf.AddRotateXYZOp((0.0, 0.0, 0.0))
        # xf.AddRotateXYZOp().Set(Gf.Vec3d(3.0, 6.0, 9.0))

    def on_destroy(self):
        carb.log_info(f"{type(self).__name__}.on_destroy()->{self.prim_path}")
        self._timeline_sub = None

    def on_play(self):
        carb.log_info(f"{type(self).__name__}.on_play()->{self.prim_path}")

    def on_pause(self):
        carb.log_info(f"{type(self).__name__}.on_pause()->{self.prim_path}")

    def on_stop(self):
        carb.log_info(f"{type(self).__name__}.on_stop()->{self.prim_path}")

    def on_update(self, current_time: float, delta_time: float):
        # carb.log_info(f"{type(self).__name__}.on_update({current_time}, {delta_time})->{self.prim_path}")
        if self._playing:
            self._rotate += delta_time * 100
            if self._rotate > 360.0:
                self._rotate = 0.0            
            rotateXYZ = (self._rotate, 0.0, self._rotate)
            quat = rotations_utils.euler_angles_to_quat(euler_angles=rotateXYZ,  # numpy.ndarray
                                                         degrees=True)  # bool
            self._prim.GetAttribute("xformOp:orient").Set(Gf.Quatd(quat[0], quat[1], quat[2], quat[3]))
            # carb.log_info(f"{type(self).__name__}.on_update: {quat},,,{rotateXYZ}")
        else:
            self._rotate = 0.0
            self._prim.GetAttribute("xformOp:orient").Set(rotateXYZ)

    def _on_timeline_event(self, e: carb.events.IEvent):
        if e.type == int(omni.timeline.TimelineEventType.PLAY):
            self._playing = True
        if e.type == int(omni.timeline.TimelineEventType.PAUSE):
            self._playing = False
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self._rotate = 0.0
            self._prim.GetAttribute("xformOp:orient").set(0.0)