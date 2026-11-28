// Private implementation fragment; included only by BedMatrix.cpp.
#ifndef BED_MATRIX_INTERPOLATION_H
#define BED_MATRIX_INTERPOLATION_H

namespace {
struct MatrixSpan {
    uint8_t low, high;
    long distance, width;

    long depth(long lowDepth, long highDepth) const {
        return width ? lowDepth + (highDepth - lowDepth) * distance / width : lowDepth;
    }
};

MatrixSpan matrixSpan(uint8_t axis, long position) {
    const float stepsPerMm = Printer::stepsPerMm(axis);
    const auto coordinate = [axis, stepsPerMm](uint8_t index) {
        const short mm = axis == X_AXIS ? g_ZCompensationMatrix[index][0] : g_ZCompensationMatrix[0][index];
        return long(mm * stepsPerMm);
    };

    uint8_t low = 1;
    long start = coordinate(low);
    if (position > start) {
        for (uint8_t high = 2; high <= g_uZMatrixMax[axis]; ++high) {
            const long end = coordinate(high);
            if (position <= end)
                return {low, high, position - start, end - start};
            low = high;
            start = end;
        }
    }
    return {low, low, 0, 0};
}
} // namespace

static long getZMatrixDepth(long x, long y) {
    const auto column = matrixSpan(X_AXIS, x);
    const auto row = matrixSpan(Y_AXIS, y);
    const long front = column.depth(g_ZCompensationMatrix[column.low][row.low], g_ZCompensationMatrix[column.high][row.low]);
    const long back = column.depth(g_ZCompensationMatrix[column.low][row.high], g_ZCompensationMatrix[column.high][row.high]);
    return row.depth(front, back);
}

#endif // BED_MATRIX_INTERPOLATION_H
