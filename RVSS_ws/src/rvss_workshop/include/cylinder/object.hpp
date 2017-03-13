/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  Cristian Rodriguez <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef OBJECT_H
#define OBJECT_H

#include <opencv2/core/core.hpp>
#include <vector>

static const uint OBJECT1 = 10;
static const uint OBJECT2 = 11;
static const uint OBJECT3 = 12;


class Object
{
private:
    double Cx;
    double Cy;
    double Cz;
    uint label;
    double invf = 1./(320.0f * tan(29.25));

    
public:
    Object(double cx, double cy, double cz, uint label){this->Cx=cx; this->Cy=cy; this->Cz=cz; this->label=label;}
    ~Object(){}
    double getCx(){return this->Cx;}
    double getCy(){return this->Cy;}
    double getCz(){return this->Cz;}
    uint getLabel(){return this->label;}

    double getZw(){return this->Cz;}
    double getXw(){return (this->getZw() * (this->Cx-320.) * this->invf);}
    double getYw(){return (this->getZw() * (this->Cy-240.) * this->invf);}
    
   
};

#endif // OBJECT_H
