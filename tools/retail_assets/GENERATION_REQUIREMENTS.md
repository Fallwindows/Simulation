# Retail asset generation requirements

The checked-in retail library has one byte-reproducible generation path. The
generator validates every prerequisite before creating or rewriting output and
raises `RuntimeError` when any prerequisite differs. It never writes reduced
fallback textures.

Review runtime:

```powershell
C:\Users\suyog\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe `
  tools\retail_assets\generate_packaging.py
```

Pinned inputs:

- Pillow `12.3.0`
- zlib runtime `1.3.2`
- `C:\Windows\Fonts\segoeui.ttf`, SHA-256
  `8134dbcd09e7b123c9a7f229d49cffbcb01352cc72ea5e1076b65d0dca9f73cd`
- `C:\Windows\Fonts\segoeuib.ttf`, SHA-256
  `aeb9e4a6ec5cc59f4d72df8189032d7dbb28f45161cf1552174818b5465dac4e`

The font binaries remain operating-system prerequisites and are not
redistributed. A host with other font revisions must stop rather than produce a
different library. The focused asset test regenerates all 79 assets with this
runtime and compares every USDA and PNG byte hash to the checked-in library.
