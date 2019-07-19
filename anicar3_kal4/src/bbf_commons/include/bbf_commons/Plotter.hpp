/*
 * Copyright 2014. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */

#pragma once
#include <string>
#include <vector>
#include <cassert>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <MRT/Commons/cOpenCvHelper.h>
#include "ColorByIndex.hpp"

typedef uint32_t ID_Type;

struct PlotPointWorld{
    float Y;
    float Z;
    Eigen::Matrix2d VarCov;
    ID_Type ID;
};
struct PlotPointPixel{
    u_int16_t U;
    u_int16_t V;
    //    u_int16_t UncertaintyU;
    //    u_int16_t UncertaintyV;
    cv::RotatedRect Ellipse;
    cv::Scalar Color;
};
/*@class struct for Label plotting
  *
*/
struct Label{
    ID_Type ID;
    std::string Text;
    //    bool Valid;

    Label()
    {
        ID=0;
        Text="";
        //        Valid=false;
    }
    Label(ID_Type CurID,std::string LabelText)
    {
        ID=CurID;
        Text=LabelText;
        //        Valid=true;
    }

};

class Plotter
{
public:
    Plotter():
        UMax(0),  //assign umAx so that we do not allocate a lot of memory and for error checking later
        VMax(0)
    {;}
    //input: area in which estimations are plotted in m (current egopose in center)
    Plotter(float WidthWindowY,float WidthWindowZ,uint16_t WindowU,uint16_t WindowV,std::string WindowName="Estimation Plotter",bool FlexibleWindow=false,std::string VerticalAxesName="X",std::string HorizontalAxesName="Z"):
        YMax(WidthWindowY),ZMax(WidthWindowZ)
      ,UMax(WindowU),VMax(WindowV)
      ,PlotMat(WindowU,WindowV,cv::Vec3b(0,0,0))
      ,WindowName(WindowName)
      ,HorizAxesName(HorizontalAxesName)
      ,VertAxesName(VerticalAxesName)
    {
        if(FlexibleWindow)  cv::namedWindow(WindowName,CV_WINDOW_NORMAL);
        else  cv::namedWindow(WindowName,CV_WINDOW_AUTOSIZE);
        CurCenterY=0.;
        CurCenterZ=0.;
        ScaleUPixPerM=UMax/YMax;
        ScaleVPixPerM=VMax/ZMax;

        MemTol=100.;

        LabelSize=std::make_pair(float(UMax)/5.,float(VMax)/20.);

    }
    virtual ~Plotter(){
        cv::destroyWindow("Estimation Plotter");
    }

    //assign new ego position and move viewing window
    /**
     * @brief set_ego_position, virtual for decoration
     * @param Y
     * @param Z
     */
    virtual void set_ego_position(float Y,float Z,std::string LabelText="Ego position"){
        if(CurCenterY!=Y || CurCenterZ!=Z)
        {
            append_to_plot(0,Y,Z,0.,0.,LabelText);
        }

        CurCenterY=Y;
        CurCenterZ=Z;
    }

    /**
     * @brief world_to_pixel
     * @param X
     * @param Scale
     * @return
     */
    inline u_int16_t world_to_pixel(float X,float Scale){
        return u_int16_t(round(Scale*X));
    }

    /**
     * @brief append_to_plot apend a point that shall be plotted, virtual for decoration
     * @param CurID
     * @param Y
     * @param Z
     * @param UncertaintyY
     * @param UncertaintyZ
     * @param LabelText
     */
    virtual void append_to_plot(ID_Type CurID, float Y, float Z, float UncertaintyY, float UncertaintyZ, std::string LabelText="")
    {
        Eigen::Matrix2d M;
        M<<UncertaintyY*UncertaintyY,0.,
                0.,UncertaintyZ*UncertaintyZ;
        append_to_plot(CurID,Y,Z,M,LabelText);
    }
    /**
 * @brief append to plot with general VarCov
 * @return  void
 * @par void
 */
    virtual void append_to_plot(ID_Type CurID, float Y, float Z, Eigen::Matrix2d VarCov, std::string LabelText="")
    {
        PlotPointWorld P;
        P.Y=Y;
        P.Z=Z;
        P.VarCov=VarCov;
        //    P.u=uint16_t(round(ScaleUPixPerM*Y));
        //    P.v=uint16_t(round(ScaleVPixPerM*Z));
        P.ID=CurID;
        MemPlot.push_back(P);

        if(LabelText.compare("")!=0)
        {
            //update existing labels as new ones
            auto Iter=std::find_if(Labels.begin(), Labels.end(), [CurID](const Label & x)
            {
                return (x.ID==CurID);
            } );

            //is new
            if (Iter==Labels.end())
            {
                Labels.push_back(Label(CurID,LabelText));
            }
        }
        //    PlotPoint P;
        //    P.u=uint16_t(round(ScaleUPixPerM*Y));
        //    P.v=uint16_t(round(ScaleVPixPerM*Z));
        //    //@todo sub with colormap
        //    P.Color=cv::Scalar(0,0,234);
    }
    /**
     * @brief convert_to_pixel
     * @param P, point in world
     * @return
     */
    PlotPointPixel convert_to_pixel(PlotPointWorld P){
        //        int MinRad=0;
        PlotPointPixel Out;
        Out.U=world_to_pixel(P.Y-CurCenterY,ScaleUPixPerM)+UMax/2;
        //        Out.UncertaintyU=world_to_pixel(P.UncertaintyY,ScaleUPixPerM)+MinRad;
        Out.V=-world_to_pixel(P.Z-CurCenterZ,ScaleVPixPerM)+VMax/2;
        //        Out.UncertaintyV=world_to_pixel(P.UncertaintyZ,ScaleVPixPerM)+MinRad;
        Out.Ellipse=getErrorEllipsePixel(P.VarCov,cv::Point2f(Out.U,Out.V));
        Out.Color=ColorByIndex::get_color(P.ID);
        return Out;
    }


    cv::RotatedRect getErrorEllipsePixel(Eigen::Matrix2d VarCov,cv::Point2f MeanPix){
        Eigen::EigenSolver<Eigen::Matrix2d> es(VarCov);
        auto EigVal=es.eigenvalues();
        auto EigVec=es.eigenvectors().col(1);

        //Calculate the angle between the largest eigenvector and the x-axis
        double angle = std::atan2(EigVec(1).real(), EigVec(0).real());

        //Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
        if(angle < 0.)
            angle += 2.*M_PI;

        //Conver to degrees instead of radians
        angle = 180.*angle/M_PI;

        //numerical issues may lead to very small negative real values
        if(std::abs(EigVal[0].real())<0.000001) EigVal[0]=0.;
        if(std::abs(EigVal[1].real())<0.000001) EigVal[1]=0.;

        //Calculate the size of the minor and major axes
        double halfmajoraxissize=std::sqrt(EigVal[0].real());
        double halfminoraxissize=std::sqrt(EigVal[1].real());

        //Return the oriented ellipse
        //The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
        return cv::RotatedRect(MeanPix, cv::Size2f(halfmajoraxissize*ScaleUPixPerM, halfminoraxissize*ScaleVPixPerM), -angle);
    }

    cv::Scalar reduce_hsv(cv::Scalar Color,cv::Vec3b ValToReduce)
    {
        cv::Mat EllipseColor(1,1,CV_8UC3,Color);
        cv::Mat NewEllipseColor(1,1,CV_8UC3);
        cv::cvtColor(EllipseColor,NewEllipseColor,CV_BGR2HSV);
        NewEllipseColor.at<Vec3b>(0,0)-=ValToReduce;
        assert(NewEllipseColor.at<Vec3b>(0,0)[1]>-1);
        cv::cvtColor(NewEllipseColor,EllipseColor,CV_HSV2BGR);
        return cv::Scalar(EllipseColor.at<Vec3b>(0,0));
    }

    void plot_scale(cv::Mat& M)
    {
        uint DistToBorder=10;
        int LineThickness=2;
        uint LengthScaleUPix=UMax/4;
        uint LengthScaleVPix=VMax/4;
        cv::Point P0(DistToBorder,DistToBorder);
        cv::Point PU(DistToBorder+LengthScaleUPix,DistToBorder);
        cv::Point PV(DistToBorder,DistToBorder+LengthScaleVPix);

        cv::Scalar col(255,255,255);
        cv::line(M,P0,PU,col,LineThickness);
        cv::line(M,P0,PV,col,LineThickness);

        float OffsetText=10;
        //@todo: no zeros after floating point
        cv::putText(M,std::to_string(std::round(float(LengthScaleUPix)/ScaleUPixPerM))+" m",P0+cv::Point(float(LengthScaleUPix/2),OffsetText+10.),cv::FONT_HERSHEY_DUPLEX, 0.5, col, LineThickness);

        cv::putText(M,std::to_string(std::round(float(LengthScaleVPix)/ScaleVPixPerM))+" m",P0+cv::Point(OffsetText,float(LengthScaleVPix/2)),cv::FONT_HERSHEY_DUPLEX, 0.5, col, LineThickness);

    }

    void plot_labels(std::vector<Label> Labels,cv::Mat& M)
    {
        float DistFromTop=10.;
        float DistFromRight=20.;
        float DistBetweenLines=10.;
        int FontFace = cv::FONT_HERSHEY_SIMPLEX;
        double DefaultScale = 1.;
        int Thickness = 1;
        uint LabelCount=0;
        for ( auto & l:Labels)
        {
            cv::Point P0(float(M.cols)-LabelSize.first-DistFromRight,
                         DistFromTop+LabelSize.second+float(LabelCount)*(DistBetweenLines+LabelSize.second));
            int baseline=0;
            //try with default scale
            cv::Size TextSize = cv::getTextSize(l.Text, FontFace,
                                                DefaultScale, Thickness, &baseline);
            double NewScaleWidth=LabelSize.first/double(TextSize.width);
            double NewScaleHeight=LabelSize.second/double(TextSize.height);
            double NewScale=NewScaleWidth<NewScaleHeight ? NewScaleWidth : NewScaleHeight;

            //            baseline += thickness;

            //                        // ... and the baseline first
            //            line(img, P0 + Point(0, thickness),
            //                 P0 + Point(textSize.width, thickness),
            //                 Scalar(0, 0, 255));
            // then put the text itself
            cv::putText(M, l.Text, P0, FontFace, NewScale*DefaultScale,
                        ColorByIndex::get_color(l.ID), Thickness);
            LabelCount++;
        }

    }
    /**
 * @brief plot axes at the origin
 */
    void plot_axes( cv::Mat& M )
    {
        PlotPointPixel PPix0,PPixY,PPixZ;
        {
            PlotPointWorld P;
            P.Y=0.;P.Z=0.;P.VarCov=Eigen::Matrix2d::Zero();P.ID=0;
            PPix0=convert_to_pixel(P);
            P.Y=10.;
            PPixY=convert_to_pixel(P);
            P.Y=0.;
            P.Z=10.;
            PPixZ=convert_to_pixel(P);
        }
        cv::Scalar col=CV_RGB(255,255,255);
        double LineThickness=2.;
        double TextDist=5.;
        cv::putText(M,VertAxesName,cv::Point(PPixY.U+TextDist,PPixY.V+TextDist),cv::FONT_HERSHEY_DUPLEX, 0.5, col, LineThickness);
        cv::line(M,cv::Point(PPix0.U,PPix0.V),cv::Point(PPixY.U,PPixY.V),col);
        cv::putText(M,HorizAxesName,cv::Point(PPixZ.U+TextDist,PPixZ.V+TextDist),cv::FONT_HERSHEY_DUPLEX, 0.5, col, LineThickness);
        cv::line(M,cv::Point(PPix0.U,PPix0.V),cv::Point(PPixZ.U,PPixZ.V),col);
    }

    /**
     * @brief plot: execute plotting virtual for decoration
     */
    virtual void plot()
    {
        assert(UMax>0. && "UMax ==0 something wrong with decorator?");
        assert(VMax>0.);
        PlotMat=cv::Mat_<cv::Vec3b>(UMax,VMax,cv::Vec3b(0,0,0));


        MemPlot.erase(std::remove_if(MemPlot.begin(),MemPlot.end(),[this](const PlotPointWorld& p)
        {
            return (abs(p.Y-CurCenterY)>YMax/2.+MemTol && abs(p.Z-CurCenterZ)>ZMax/2.+MemTol);
        } ),MemPlot.end());

        PlotPointPixel P;
        //        std::vector<Label> ShowIDs;
        for ( auto & el:MemPlot)
        {
            P=convert_to_pixel(el);
            if (P.U>0 && P.U<PlotMat.cols && P.V>0 && P.V<PlotMat.rows) {
                cv::circle(PlotMat, cv::Point(P.U,P.V),2.,P.Color,2);

                cv::ellipse(PlotMat, P.Ellipse, reduce_hsv(P.Color,cv::Vec3b(0,120,80)),1); //threshold for uncertainty for track to be shown is not set here but in UKF.Filter
            }

            //            //add labels to plot
            //                    auto& Iter=std::find_if(Labels.begin(), Labels.end(), [](const Label & x)
            //                    {
            //                        return x.ID==ID;
            //                    } );

            //                    //is found
            //                    if (Iter!=Labels.end())
            //                    {
            //                        ShowIDs.push_back(*Iter);
            //                    }

        }
        //plot scale
        plot_scale(PlotMat);

        plot_labels(Labels,PlotMat);

        plot_axes(PlotMat);

        cv::imshow(WindowName,PlotMat);
        cv::waitKey(3);

        //reset
        Labels=std::vector<Label>();
    }
    void clear()
    {
        MemPlot.clear();
        MemPlot.reserve(0);
    }



public: //attributes
    ///@brief WindowName
    std::string WindowName;
    ///@brief Mat on which stuff will be plotted
    cv::Mat_<cv::Vec3b> PlotMat;
    ///@brief storage for all values that will be plotted again
    std::vector<PlotPointWorld> MemPlot;
    ///@brief YMax*ZMax is area which is plotted
    float YMax,ZMax;
    ///@brief plot center in world cooridnates in m
    float CurCenterY,CurCenterZ;
    ///@brief size of plotting window
    uint16_t UMax,VMax;
    ///@brief Scale resulting from UmAx and YMax and ZV, entity: pixel per meter
    float ScaleUPixPerM,ScaleVPixPerM;
    ///@brief Tolerance in which points that are not seen are not thrown away in m
    float MemTol;
    ///@brief Label width and height
    std::pair<double,double> LabelSize;
    ///@brief Labels
    std::vector<Label> Labels;
    ///@brief name of vertical and horizontal axes at origin
    std::string VertAxesName,HorizAxesName;
};
