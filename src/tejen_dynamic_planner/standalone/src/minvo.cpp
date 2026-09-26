#include "dynamic_planner/minvo.hpp"

#include <stdexcept>

namespace dynamic_planner {
namespace {

Matrix4 matrix4FromRows(const double (&v)[4][4]) {
    Matrix4 m;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            m(r, c) = v[r][c];
        }
    }
    return m;
}

Matrix3 matrix3FromRows(const double (&v)[3][3]) {
    Matrix3 m;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            m(r, c) = v[r][c];
        }
    }
    return m;
}

const Matrix4& mSeg0() {
    static const double v[4][4] = {
        {1.1023313949144333268, 0.34205724556666972092, -0.09273093424558287445, -0.03203276669713062130},
        {-0.04968355625374917817, 0.65780347324677179710, 0.53053863760186903420, 0.21181027098212013016},
        {-0.04730904421116234604, 0.01559443689415558609, 0.50518275571593496132, 0.63650059656260427055},
        {-0.00533879444952174449, -0.01545515570759708329, 0.05700954092777830301, 0.18372189915240558222},
    };
    static const Matrix4 m = matrix4FromRows(v);
    return m;
}

const Matrix4& mSeg1() {
    static const double v[4][4] = {
        {0.27558284872860833170, 0.08551431139166743023, -0.02318273356139571861, -0.00800819167428265533},
        {0.60990427619758658118, 0.63806904207840509091, 0.29959938009132258685, 0.12252106674808682651},
        {0.11985166952332682033, 0.29187180223752445807, 0.66657381254229419731, 0.70176522577378930290},
        {-0.00533879444952174449, -0.01545515570759708329, 0.05700954092777830301, 0.18372189915240558222},
    };
    static const Matrix4 m = matrix4FromRows(v);
    return m;
}

const Matrix4& mRest() {
    static const double v[4][4] = {
        {0.18372189915240555447, 0.05700954092777830995, -0.01545515570759711799, -0.00533879444952181648},
        {0.70176522577378919188, 0.66657381254229419731, 0.29187180223752384745, 0.11985166952332582113},
        {0.11985166952332682033, 0.29187180223752445807, 0.66657381254229419731, 0.70176522577378930290},
        {-0.00533879444952174449, -0.01545515570759708329, 0.05700954092777830301, 0.18372189915240558222},
    };
    static const Matrix4 m = matrix4FromRows(v);
    return m;
}

const Matrix4& mLast2() {
    static const double v[4][4] = {
        {0.18372189915240569325, 0.05700954092777830995, -0.01545515570759714574, -0.00533879444952181648},
        {0.70176522577378952494, 0.66657381254229453038, 0.29187180223752412500, 0.11985166952332593215},
        {0.12252106674808753428, 0.29959938009132280889, 0.63806904207840497989, 0.60990427619758624811},
        {-0.00800819167428261543, -0.02318273356139562147, 0.08551431139166744411, 0.27558284872860833170},
    };
    static const Matrix4 m = matrix4FromRows(v);
    return m;
}

const Matrix4& mLast() {
    static const double v[4][4] = {
        {0.18372189915240555447, 0.05700954092777830995, -0.01545515570759711799, -0.00533879444952181648},
        {0.63650059656260415952, 0.50518275571593496132, 0.01559443689415529466, -0.04730904421116288727},
        {0.21181027098212068527, 0.53053863760186914522, 0.65780347324677146403, -0.04968355625374962226},
        {-0.03203276669713046171, -0.09273093424558248588, 0.34205724556666977643, 1.10233139491443332680},
    };
    static const Matrix4 m = matrix4FromRows(v);
    return m;
}

const Matrix3& mVelSeg0() {
    static const double v[3][3] = {
        {1.077349059083916, 0.1666702138890985, -0.07735049175615138},
        {-0.03867488648729411, 0.7499977187062712, 0.5386802643920123},
        {-0.03867417280506149, 0.08333206631563977, 0.538670227146185},
    };
    static const Matrix3 m = matrix3FromRows(v);
    return m;
}

const Matrix3& mVelRest() {
    static const double v[3][3] = {
        {0.538674529541958, 0.08333510694454926, -0.03867524587807569},
        {0.4999996430546639, 0.8333328256508203, 0.5000050185139366},
        {-0.03867417280506149, 0.08333206631563977, 0.538670227146185},
    };
    static const Matrix3 m = matrix3FromRows(v);
    return m;
}

const Matrix3& mVelLast() {
    static const double v[3][3] = {
        {0.538674529541958, 0.08333510694454926, -0.03867524587807569},
        {0.5386738158597254, 0.7500007593351806, -0.03866520863224832},
        {-0.07734834561012298, 0.1666641326312795, 1.07734045429237},
    };
    static const Matrix3 m = matrix3FromRows(v);
    return m;
}

}  // namespace

std::vector<Matrix4> positionConverters(int num_segments) {
    if (num_segments < 4) {
        throw std::invalid_argument("released MADER cubic MINVO converter expects at least 4 segments");
    }
    std::vector<Matrix4> matrices;
    matrices.reserve(static_cast<std::size_t>(num_segments));
    matrices.push_back(mSeg0());
    matrices.push_back(mSeg1());
    for (int i = 0; i < num_segments - 4; ++i) {
        matrices.push_back(mRest());
    }
    matrices.push_back(mLast2());
    matrices.push_back(mLast());
    return matrices;
}

std::vector<Matrix3> velocityConverters(int num_segments) {
    if (num_segments < 2) {
        throw std::invalid_argument("released RMADER MINVO velocity converter expects at least 2 segments");
    }
    std::vector<Matrix3> matrices;
    matrices.reserve(static_cast<std::size_t>(num_segments));
    matrices.push_back(mVelSeg0());
    for (int i = 0; i < num_segments - 2; ++i) {
        matrices.push_back(mVelRest());
    }
    matrices.push_back(mVelLast());
    return matrices;
}

FourPoints minvoVertices(const FourPoints& bspline_control_points,
                         int interval_index,
                         int num_segments) {
    const auto converters = positionConverters(num_segments);
    if (interval_index < 0 || interval_index >= static_cast<int>(converters.size())) {
        throw std::out_of_range("interval_index out of range");
    }
    return converters.at(static_cast<std::size_t>(interval_index)).transpose()
           * bspline_control_points;
}

ThreePoints minvoVelocityVertices(const ThreePoints& velocity_control_points,
                                  int interval_index,
                                  int num_segments) {
    const auto converters = velocityConverters(num_segments);
    if (interval_index < 0 || interval_index >= static_cast<int>(converters.size())) {
        throw std::out_of_range("interval_index out of range");
    }
    return converters.at(static_cast<std::size_t>(interval_index)).transpose()
           * velocity_control_points;
}

}  // namespace dynamic_planner
