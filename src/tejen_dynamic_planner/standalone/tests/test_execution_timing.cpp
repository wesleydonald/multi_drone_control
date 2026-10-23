#include "dynamic_planner/execution_timing.hpp"
#include "test_common.hpp"
#include <cmath>
#include <limits>

using namespace dynamic_planner;
int main() {
    requireTrue(!strictlyBeforeSplice(.3, .3), "equal splice admitted");
    requireTrue(!strictlyBeforeSplice(.3, std::nextafter(.3, INFINITY)), "one ULP admitted");
    const double epoch = 1791710000.;
    requireTrue(!strictlyBeforeSplice(epoch, std::nextafter(epoch, INFINITY)), "wall-time ULP admitted");
    requireTrue(strictlyBeforeSplice(epoch, epoch + .01), "real future splice rejected");
    requireTrue(!strictlyBeforeSplice(NAN, .3), "invalid clock admitted");
    requireNear(adaptiveSpliceLookahead(0., 1./30., 2.5, .05, 1.), 2.5/30., 1e-12, "zero history changed");
    requireNear(adaptiveSpliceLookahead(.09, 1./30., 2.5, .05, 1.), .25, 1e-12, "adaptive equation changed");
}
