NDWarp Plugin
=============

Plugin for [areaDetector](https://areadetector.github.io/master/index.html)
which performs coordinate transformation of images.

See [NDWarp.template](warpApp/Db/NDWarp.template) for names.

The currently implemented transformation is a rotation about a center
(`"$(P)$(R)CenterX"` and `"$(P)$(R)CenterXY`) by an arbitrary angle (`$(P)$(R)Angle`).

Optionally, a second order scaling (`$(P)$(R)FactorX` and `$(P)$(R)FactorY`) can be
given.  eg. to effect a [Scheimpflug transformation](https://en.wikipedia.org/wiki/Scheimpflug_principle).
See [NDWarp-frib-angle.db](warpApp/Db/NDWarp-frib-angle.db).

For exact expressions, see `fill_mapping()` in [NDPluginWarp.cpp](warpApp/src/NDPluginWarp.cpp).
