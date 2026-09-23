# 录制帧夹具来源（`test/recordings/`）

这些文件是**录下来的真实帧**（不是合成图），供 `test_camera_timestamp.py` 做拼接帧切分/格式的
回放测试。它们只是数据，**不携带任何曝光时间戳**。

| 文件 | 字节数 | sha256 |
| --- | --- | --- |
| `stereo.png` | 509211 | `a2ee0a8110901b75ca85b6e9102c232b31d170a5e167dd1afa1028b7ea630e2e` |
| `half_a.png` | 268254 | `87c96c726115aa258d5ababb425d88ae4c2e77f0dc4da3491e1f2b4cf410ed61` |
| `half_b.png` | 239745 | `8933a50c7e7a98a09e3369431a54a7d4b1156a437e10c49371f52c29c01a4686` |

来源（Windows 资料目录，只读复制，未裁剪、未重编码、未做任何处理）：

    data/stereo_depth/windows_snapshot_20260922/captures/20260911_200503_356811/

原目录另有一份 `capture.json`，**未复制**（本目录只放测试实际用到的三张图）。

## 事实与边界

- 尺寸：`stereo.png` 为 480×1280 的整幅拼接帧，`half_a.png` / `half_b.png` 各为 480×640。
- 实测（本机 2026-09-23，OpenCV 4.6.0）：`stereo.png` 解出的左半/右半与 `half_a.png` /
  `half_b.png` **逐像素完全一致**（PNG 无损）；两张半图彼此不同，因此「切错列」可被发现。
- 原目录 `capture.json` 写明
  `"timestamp_note": "directory time is host save time, not sensor exposure time"`；
  同一批次的另一些目录写作 `"timestamp": "host save time, not exposure time"`。
  即：录制帧**没有曝光时间戳**，测试里出现的单调时间戳是测试自己造的占位值。
- 因此这些夹具只能证明「一整幅拼接帧 → 整幅解码 → 按宽度对半切」的**格式/切分**逻辑，
  **不能证明曝光时刻**，也不构成真机时间戳验收（需硬件标定，见包 README 的待验收清单）。