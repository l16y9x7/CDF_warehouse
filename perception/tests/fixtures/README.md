`barcode_tilted_21b789.png` is the unmodified 139×295 crop from request
`2026-10-01/21b789c2348d4a93995a670d5f242846`, expanded bbox
`[591, 157, 730, 452]`. Expected EAN-13: `3282779003131`.

The original image SHA-256 is
`55324ef1057c74beeecb79fdad0b7b8d2b8237bd255c6807036594ccb666d0ce`.
ZXing-C++ 3.1.1 misses the native crop but decodes it after a +25° rotation.
This fixture prevents the rotation fallback from regressing; no SAM3 call is needed.
