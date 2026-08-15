#pragma once
#include <cstdint>
#include "model/render_snapshot.h"

// ---------------------------------------------------------- ui fix panel --
enum class PositionView : uint8_t { LIVE, TRIP };
extern PositionView positionView;

void drawFixPanel(const RenderSnapshot &s);
