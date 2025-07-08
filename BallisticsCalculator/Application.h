#pragma once
#include "Plotter.h"

extern void AppInit();
extern void AppUpdate();
//TODO: change this to pass in curve information instead? Or separate curves from this entirely?
extern void AppHitDelegate(const Algebra::Vector2D& Point, Plotter::Curve2D::MetaDataTagType Tag);