"""Reusable OmniVLA predictor supporting the non-satellite goal modalities.

Place this file next to inference/run_omnivla.py, e.g.:
    OmniVLA/inference/omnivla_predictor.py

The original run_omnivla.py is not modified. This wrapper deliberately reuses
its model-loading, preprocessing, and forward-pass implementation so tests use
the same path as the authors' example.

Supported modalities:
    pose           -> modality_id 4
    pose_image     -> modality_id 5
    image          -> modality_id 6
    language       -> modality_id 7
    language_pose  -> modality_id 8

Satellite modes (0-3) are intentionally not exposed here yet.
"""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
from typing import Optional, Sequence, Union

import numpy as np
from PIL import Image

try:
    # Works when imported as inference.omnivla_predictor.
    from . import run_omnivla as omni
except ImportError:
    # Works when running `python inference/test_*.py` from the repo root.
    import run_omnivla as omni


ImageLike = Union[str, Path, Image.Image]


MODALITY_FLAGS = {
    "pose": dict(pose_goal=True, image_goal=False, lan_prompt=False),
    "pose_image": dict(pose_goal=True, image_goal=True, lan_prompt=False),
    "image": dict(pose_goal=False, image_goal=True, lan_prompt=False),
    "language": dict(pose_goal=False, image_goal=False, lan_prompt=True),
    "language_pose": dict(pose_goal=True, image_goal=False, lan_prompt=True),
}


@contextmanager
def _temporary_numpy_seed(seed: int):
    """Make the author's dummy action-token scaffold deterministic.

    run_omnivla.py currently creates np.random.rand(8, 4) dummy actions while
    constructing action-token positions. We preserve that behavior, but fix the
    RNG during preprocessing so repeated sanity checks are comparable.
    """
    state = np.random.get_state()
    np.random.seed(seed)
    try:
        yield
    finally:
        np.random.set_state(state)


def _to_rgb_pil(image: ImageLike) -> Image.Image:
    if isinstance(image, Image.Image):
        return image.convert("RGB")
    return Image.open(Path(image)).convert("RGB")


class OmniVLAPredictor:
    """Thin reusable wrapper around the authors' OmniVLA inference path.

    The model always receives the current egocentric RGB observation. Depending
    on `modality`, the active goal information is a goal image, relative pose,
    language instruction, or a supported combination.

    Returned trajectory shape is normally (8, 4), with each row:
        [x, y, heading_x, heading_y]

    x/y remain in the model's normalized waypoint units. The authors' standalone
    inference script multiplies x/y by 0.1 before its controller; that scaling is
    intentionally kept outside this class.
    """

    def __init__(
        self,
        model_path: str = "./omnivla-original",
        resume_step: int = 120000,
        dummy_action_seed: int = 0,
    ) -> None:
        # Set a valid initial mode before loading. Per-call flags are reasserted
        # in predict() because run_forward_pass() reads module globals.
        self._set_modality_globals("image")

        cfg = omni.InferenceConfig()
        cfg.vla_path = model_path
        cfg.resume_step = resume_step
        cfg.resume = True
        cfg.use_l1_regression = True
        cfg.use_diffusion = False
        cfg.use_film = False
        cfg.num_images_in_input = 2

        (
            self.vla,
            self.action_head,
            self.pose_projector,
            self.device_id,
            self.num_patches,
            self.action_tokenizer,
            self.processor,
        ) = omni.define_model(cfg)

        # Reuse preprocessing and forward-pass helpers only. The helper's own
        # GPS/current-state loop is never called.
        dummy_goal = Image.new("RGB", (2, 2), color=0)
        self._helper = omni.Inference(
            save_dir="./inference",
            lan_inst_prompt="",
            goal_utm=(0.0, 0.0, 0, "N"),
            goal_compass=0.0,
            goal_image_PIL=dummy_goal,
            action_tokenizer=self.action_tokenizer,
            processor=self.processor,
        )
        self.dummy_action_seed = int(dummy_action_seed)

    @staticmethod
    def make_goal_pose(
        forward_m: float,
        left_m: float,
        yaw_deg: float = 0.0,
        waypoint_spacing: float = 0.1,
    ) -> np.ndarray:
        """Create OmniVLA's 4-D relative-pose representation.

        Parameters
        ----------
        forward_m:
            Goal displacement forward in the robot frame.
        left_m:
            Goal displacement to the robot's left. Use a negative value for right.
        yaw_deg:
            Relative goal heading in degrees. Positive means left/CCW.
        waypoint_spacing:
            Position normalization used by the standalone inference script.
            Default 0.1 m.

        Returns
        -------
        np.ndarray, shape (4,)
            [forward/spacing, left/spacing, cos(yaw), sin(yaw)]
        """
        if waypoint_spacing <= 0:
            raise ValueError("waypoint_spacing must be positive.")

        yaw = np.deg2rad(float(yaw_deg))
        return np.asarray(
            [
                float(forward_m) / float(waypoint_spacing),
                float(left_m) / float(waypoint_spacing),
                np.cos(yaw),
                np.sin(yaw),
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _validate_goal_pose(goal_pose: Sequence[float]) -> np.ndarray:
        arr = np.asarray(goal_pose, dtype=np.float32)
        if arr.shape != (int(omni.POSE_DIM),):
            raise ValueError(
                f"goal_pose must have shape ({omni.POSE_DIM},), got {arr.shape}"
            )
        return arr

    @staticmethod
    def _set_modality_globals(modality: str) -> None:
        if modality not in MODALITY_FLAGS:
            raise ValueError(
                f"Unsupported modality '{modality}'. "
                f"Choose one of: {', '.join(MODALITY_FLAGS)}"
            )

        flags = MODALITY_FLAGS[modality]
        omni.satellite = False
        omni.pose_goal = flags["pose_goal"]
        omni.image_goal = flags["image_goal"]
        omni.lan_prompt = flags["lan_prompt"]

    def predict(
        self,
        current_image: ImageLike,
        goal_image: Optional[ImageLike] = None,
        *,
        modality: str = "image",
        goal_pose: Optional[Sequence[float]] = None,
        language: Optional[str] = None,
    ) -> np.ndarray:
        """Predict one local trajectory.

        Parameters
        ----------
        current_image:
            Current egocentric RGB observation.
        goal_image:
            Goal RGB image. Required for `image` and `pose_image`.
        modality:
            One of: pose, pose_image, image, language, language_pose.
        goal_pose:
            Normalized 4-D OmniVLA pose representation. Required for `pose`,
            `pose_image`, and `language_pose`. Use `make_goal_pose()` for the
            convenient metre/degree interface.
        language:
            Language command. Required for `language` and `language_pose`.

        Returns
        -------
        np.ndarray, shape (NUM_ACTIONS_CHUNK, ACTION_DIM), normally (8, 4)
        """
        if modality not in MODALITY_FLAGS:
            raise ValueError(
                f"Unsupported modality '{modality}'. "
                f"Choose one of: {', '.join(MODALITY_FLAGS)}"
            )

        flags = MODALITY_FLAGS[modality]

        if flags["image_goal"] and goal_image is None:
            raise ValueError(f"modality='{modality}' requires goal_image.")
        if flags["pose_goal"] and goal_pose is None:
            raise ValueError(f"modality='{modality}' requires goal_pose.")
        if flags["lan_prompt"] and (language is None or not language.strip()):
            raise ValueError(f"modality='{modality}' requires a non-empty language instruction.")

        current_pil = _to_rgb_pil(current_image)

        # The current OmniVLA preprocessing/model path is configured for two
        # images. For non-image-goal modalities, provide current RGB again as a
        # placeholder; modality_id tells the model which goal modalities are active.
        goal_pil = (
            _to_rgb_pil(goal_image)
            if goal_image is not None
            else current_pil.copy()
        )

        pose = (
            self._validate_goal_pose(goal_pose)
            if goal_pose is not None
            else np.zeros((int(omni.POSE_DIM),), dtype=np.float32)
        )

        # transform_datatype() uses "xxxx" as the no-language marker.
        language_instruction = language.strip() if flags["lan_prompt"] else "xxxx"

        # run_forward_pass() reads these module globals to choose modality_id.
        self._set_modality_globals(modality)

        with _temporary_numpy_seed(self.dummy_action_seed):
            batch = self._helper.data_transformer_omnivla(
                current_pil,
                language_instruction,
                goal_pil,
                pose,
                prompt_builder=omni.PurePromptBuilder,
                action_tokenizer=self.action_tokenizer,
                processor=self.processor,
            )

        # Reassert immediately before the forward pass in case external code
        # touched the module globals between preprocessing and inference.
        self._set_modality_globals(modality)

        actions, modality_id = self._helper.run_forward_pass(
            vla=self.vla.eval(),
            action_head=self.action_head.eval(),
            noisy_action_projector=None,
            pose_projector=self.pose_projector.eval(),
            batch=batch,
            action_tokenizer=self.action_tokenizer,
            device_id=self.device_id,
            use_l1_regression=True,
            use_diffusion=False,
            use_film=False,
            num_patches=self.num_patches,
            compute_diffusion_l1=False,
            num_diffusion_steps_train=None,
            mode="vali",
            idrun=0,
        )

        trajectory = actions.detach().float().cpu().numpy()
        if trajectory.ndim != 3 or trajectory.shape[0] != 1:
            raise RuntimeError(
                f"Expected OmniVLA output [1, horizon, action_dim], got {trajectory.shape}"
            )

        self.last_modality_id = int(modality_id.detach().cpu().reshape(-1)[0].item())
        return trajectory[0]
