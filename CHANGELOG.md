# Changelog for package mono3d_indoor_detection

tros_1.1.3 (2022-10-09)
------------------
1. 头文件中删除不必要的类型引用。


hhp_1.0.6 (2022-08-25)
------------------
1. 支持通过shared mem方式订阅图片，用于算法推理。
2. 支持在板端渲染发布的AI消息。
3. 支持运行时通过配置选择是否在板端渲染AI消息并保存渲染后的图片。
4. 增加mono3d_indoor_detection_pipeline.launch.py启动脚本文件，支持通过WEB端展示算法效果以及选择订阅图片的输入源。


hhp_1.0.4 (2022-07-26)
------------------
1. 适配hhp_1.0.4版本dnn_node。
