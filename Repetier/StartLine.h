#ifndef START_LINE_H
#define START_LINE_H

class GCode;

class StartLine {
public:
    bool active() const { return active_; }
    void run(GCode& command);

private:
    bool active_ = false;
};

#endif
