/*--
    Debugging.cpp  

    This file is part of the Cornucopia curve sketching library.
    Copyright (C) 2010 Ilya Baran (baran37@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>

#include "Algorithm.h"
#include "CurvePrimitive.h"
#include "Debugging.h"
#include "Polyline.h"

using namespace std;
using namespace Eigen;
NAMESPACE_Cornu

Debugging *Debugging::_currentDebugging = new Debugging();

void Debugging::set(Debugging *debugging)
{
    delete _currentDebugging;
    _currentDebugging = debugging;
}

void Debugging::drawPrimitive(CurvePrimitiveConstPtr curve, const std::string &group, int idx, double thickness)
{
    Color color = Color::Ones() * ((idx & 1) ? 0 : 0.5);
    double val = (idx & 1) ? 0.7 : 1.;
    color[curve->getType()] = val;

    drawCurve(curve, color, group, thickness);
}

// more or less copy&paste from private function Cornu::Document::_writeNative()
void writeJson(const PolylineConstPtr& line, const Parameters& params, std::ostream& stream) {
    stream << "[\n";

    stream << "{ ";

    // write out coordinates
    stream << "\"pts\" : [ ";

    const Cornu::VectorC<Eigen::Vector2d>& pts = line->pts();
    for (int j = 0; j < pts.size(); ++j) {
        if (j > 0)
            stream << " , ";
        stream << pts[j][0] << ", " << pts[j][1];
    }

    stream << " ] ";

    // write out parameters
    for (int j = 0; j < (int)Cornu::Parameters::parameters().size(); ++j) {
        const Cornu::Parameters::Parameter& param = Cornu::Parameters::parameters()[j];
        stream << " ,\n      \"" << param.typeName.c_str() << "\" : ";
        stream << params.get(param.type);
    }

    // write out algorithms
    for (int j = 0; j < Cornu::NUM_ALGORITHM_STAGES; ++j) {
        Cornu::AlgorithmBase* alg =
            Cornu::AlgorithmBase::get((Cornu::AlgorithmStage)j, params.getAlgorithm(j));
        stream << " ,\n      \"" << alg->stageName().c_str() << "\" : ";
        stream << "\"" << alg->name().c_str() << "\"";
    }

    // write out oversketch index
    stream << " ,\n      \"oversketch\" : " << -1;

    stream << " }";

    stream << " ]";
}

END_NAMESPACE_Cornu
