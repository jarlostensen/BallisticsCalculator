#pragma once
#include "Plotter.h"

extern void AppInit();
extern void AppUpdate();
extern void AppHitDelegate(const Algebra::Vector2D& Point, Plotter::Curve2D::MetaDataTagType Tag);