#include <iostream>
#include <string>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

using namespace std;

const string OUT_DIR = "/home/suhailps/Desktop/winter_project/Pointclouds/";
double xmin, ymin, zmin, xmax,ymax,zmax;

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer ("PCL Viewer")
    {
                frames_saved = 0;
                // save_one = false;
    }

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
                if (!viewer.wasStopped()) {
                    // PCL_INFO ("Showing off.");

                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


                            copyPointCloud  (*cloud,*input_cloud);

                          pcl::PassThrough<pcl::PointXYZRGB> pass;
                          pass.setInputCloud (input_cloud);
                          pass.setFilterFieldName ("x");
                          pass.setFilterLimits (xmin, xmax);
                          pass.filter (*input_cloud);
                          viewer.showCloud (input_cloud);

                          pass.setInputCloud (input_cloud);
                          pass.setFilterFieldName ("y");
                          pass.setFilterLimits (ymin, ymax);
                          pass.filter (*input_cloud);
                          viewer.showCloud (input_cloud);



                          pass.setInputCloud (input_cloud);
                          pass.setFilterFieldName ("z");
                          pass.setFilterLimits (zmin, zmax);
                          pass.filter (*input_cloud);
                          viewer.showCloud (input_cloud);

                        if( save_one ) {
                                // save_one = false;
                                std::stringstream out;
                                out << frames_saved;
                                std::string name = OUT_DIR + "cloud" + out.str() + ".pcd";
                                cout << "Saving frame @" << name << ".\n";
                                frames_saved++;
                                pcl::io::savePCDFileASCII( name, *input_cloud );
                        }
                }
    }

    void run ()
    {
                pcl::Grabber* interface = new pcl::OpenNIGrabber();

                boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
                        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

                interface->registerCallback (f);

                interface->start ();

                char c;

                while (!viewer.wasStopped())
                {
                        //sleep (1);

                    cout << "enter c for changing crop box dimensions and s for starting Saving  ";

                        c = getchar();
                        if( c == 's' ) {
                                cout << "Saving frame " << frames_saved << ".\n";
                                frames_saved++;
                                save_one = true;
                        }

                        else if( c == 'x' ) {
                                cout << "Saving stopped .\n";
                                save_one = false;
                        }

                        else if(c=='c')
                        {

                            cout << "Enter xmin xmax ymin ymax zmin zmax \n";
                            cin >> xmin>> xmax>> ymin >>ymax>> zmin>> zmax;
                            cout << " xmin= "<<xmin<< " xmax= "<<xmax<< " ymin= "<<ymin <<" ymax= "<<ymax<<" zmin= " << zmin<<" zmax= "<< zmax<< ".\n";


                        }
                }

                interface->stop ();
        }

        pcl::visualization::CloudViewer viewer;

        private:
                int frames_saved;
                bool save_one;

};

int main ()
{
     xmin=-1000; ymin= -1000;zmin= -1000;
 xmax=1000; ymax= 1000;zmax=1000;
    SimpleOpenNIViewer v;
    v.run ();
    return 0;
}
