import base64
from dataclasses import dataclass

from agent.capabilities.camera import CameraCapability
from agent.capabilities.common import CapabilityError, Hand
from agent.capabilities.manipulation import ManipulationCapability
from agent.capabilities.perception import PerceptionCapability, RecognizeBarcodeRequest
from agent.contracts import ExecutionContext
from agent.skills.base import SkillError, action_id, emit_failed, emit_started, emit_succeeded, fail


@dataclass(frozen=True)
class RecognizeBarcodeInput:
    expected_sku_id: str | None
    name: str | None
    hand: Hand
    sku_typ: str


@dataclass(frozen=True)
class RecognizeBarcodeResult:
    sku_id: str


class RecognizeAndVerifyBarcodeSkill:
    name = "recognize_and_verify_barcode"
    version = "5"

    def __init__(
        self,
        perception: PerceptionCapability,
        camera: CameraCapability,
        manipulation: ManipulationCapability,
    ):
        self.perception, self.camera, self.manipulation = perception, camera, manipulation

    def execute(
        self, context: ExecutionContext, data: RecognizeBarcodeInput
    ) -> RecognizeBarcodeResult:
        key = action_id(context, self.name)
        emit_started(context, self.name, action_id=key)
        try:
            scan = self.manipulation.rotate(data.hand, data.sku_typ, idempotency_key=key)
            content, image_index = self._recognize_scan(scan.image_paths, data)
        except SkillError as exc:
            emit_failed(context, self.name, exc, action_id=key)
            raise
        except OSError as exc:
            stable = SkillError("SCAN_IMAGE_UNAVAILABLE", "barcode scan image is unavailable")
            emit_failed(context, self.name, stable, action_id=key)
            raise stable from exc
        except CapabilityError as exc:
            if exc.error_code in {"SKU_BARCODE_NOT_FOUND", "SKU_BARCODE_MISMATCH"}:
                stable = SkillError(exc.error_code, exc.message)
                emit_failed(context, self.name, stable, action_id=key)
                raise stable from exc
            emit_failed(context, self.name, exc, action_id=key)
            raise fail(exc) from exc
        except Exception as exc:
            emit_failed(context, self.name, exc, action_id=key)
            raise fail(exc) from exc
        emit_succeeded(
            context,
            self.name,
            sku_id=content,
            action_id=key,
            camera=scan.camera,
            image_count=len(scan.image_paths),
            matched_image_index=image_index,
        )
        return RecognizeBarcodeResult(content)

    def _recognize_scan(
        self, image_paths: tuple[str, ...], data: RecognizeBarcodeInput
    ) -> tuple[str, int]:
        found_other_barcode = False
        for index, path in enumerate(image_paths, start=1):
            try:
                content = self._recognize_image(path, data)
            except CapabilityError as exc:
                if exc.error_code == "SKU_BARCODE_NOT_FOUND":
                    continue
                raise
            if data.expected_sku_id is None or content == data.expected_sku_id:
                return content, index
            found_other_barcode = True
        if found_other_barcode:
            raise SkillError("SKU_BARCODE_MISMATCH", "barcode does not match expected SKU")
        raise SkillError("SKU_BARCODE_NOT_FOUND", "SKU barcode was not found")

    def _recognize_image(self, path: str, data: RecognizeBarcodeInput) -> str:
        image_base64 = base64.b64encode(self.camera.read_image_bytes(path)).decode("ascii")
        result = self.perception.recognize_sku_barcode(
            RecognizeBarcodeRequest(
                image_base64, data.expected_sku_id or "", data.name or ""
            )
        )
        return result.barcode_content


__all__ = ["RecognizeAndVerifyBarcodeSkill", "RecognizeBarcodeInput", "RecognizeBarcodeResult"]
