#ifndef GCODE_MODES_H
#define GCODE_MODES_H

class GCodeModes {
public:
    bool relativeXYZ() const { return relativeXYZ_; }
    void setRelativeXYZ(bool relative) { relativeXYZ_ = relative; }

    bool relativeEOverride() const { return relativeEOverride_; }
    void setRelativeEOverride(bool relative) { relativeEOverride_ = relative; }

    bool relativeE() const { return relativeXYZ_ || relativeEOverride_; }

private:
    bool relativeXYZ_ = false;
    bool relativeEOverride_ = false;
};

#endif
