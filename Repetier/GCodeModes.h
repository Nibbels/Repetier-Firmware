#ifndef GCODE_MODES_H
#define GCODE_MODES_H

class GCodeModes {
public:
    bool inches() const { return inches_; }
    void setInches(bool inches) { inches_ = inches; }

    bool relativeXYZ() const { return relativeXYZ_; }
    void setRelativeXYZ(bool relative) { relativeXYZ_ = relative; }

    bool relativeEOverride() const { return relativeEOverride_; }
    void setRelativeEOverride(bool relative) { relativeEOverride_ = relative; }

    bool relativeE() const { return relativeXYZ_ || relativeEOverride_; }

private:
    bool inches_ = false;
    bool relativeXYZ_ = false;
    bool relativeEOverride_ = false;
};

#endif
