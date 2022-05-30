# hrpsys_ext_rtc

このrtcを使用するためには、[hrpsys_tools](http://wiki.ros.org/hrpsys_tools)は外部のRTCをプラグインとしてロードする機能に乏しいので、代わりに[rtcloader](https://github.com/Naoki-Hiraoka/rtcloader)を使用してください。

- SoftErrorLimiter2: https://github.com/fkanehiro/hrpsys-base/tree/master/rtc/SoftErrorLimiter とは、https://github.com/fkanehiro/hrpsys-base/pull/1294 を反映していて、choreonoidモデルを使用している点が異なる.