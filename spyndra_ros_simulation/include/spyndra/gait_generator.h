#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <spyndra/util.h>


#ifndef SPYNDRA_GAIT_GENERATOR_H
#define SPYNDRA_GAIT_GENERATOR_H
namespace spyndra
{
class GaitGenerator
{
public:
  GaitGenerator(std::string method);
  GaitGenerator(std::string method, std::string filename);
  ~GaitGenerator(){ delete impl_;}
  void set_controllers(const ros::V_Publisher & v ){ impl_->ControllerPtr = &v; }
  void next_step(); 
  void print_gait(){ impl_->print_gait_impl(); }
  bool is_over(){ impl_->is_over_impl(); }


private:
  /* Abstract Implementation */
  class Impl
  {
  public:
    Impl() {}
    ~Impl() {}
    virtual void next_step_impl( ) = 0;
    virtual bool is_over_impl( ) = 0;

    // TODO: figure out how to make it final
    void print_gait_impl();

    const ros::V_Publisher* ControllerPtr;
    std::list<std::vector<double> > gait;
  };
  typedef Impl* ImplPtr;

  class CsvImpl : public Impl
  {
  public:
    CsvImpl(){}
    ~CsvImpl(){}
    void next_step_impl(); //{std::cout <<"Csv next\n";}
    bool is_over_impl();
    void set_csv_file( std::string filename );

  private:
    std::list<std::vector<double> >::iterator curr;
  };

  /*
  class RandImpl : public Impl
  {
  public:
    RandImpl(){}
    ~RandImpl(){}
    void next_step_impl(){std::cout << "Rand next\n";}
  };

  class SplineImpl : public Impl
  {
  public:
    SplineImpl(){}
    ~SplineImpl(){}
    void next_step_impl(){std::cout << "Spline next\n";}
    void set_csv_file(const std::string& filename )
    { std::cout << "Set file " << filename << '\n'; }
  };
  */

  /* Member variable */
  ImplPtr impl_;
  bool is_valid() { return impl_; }
};


/* gait_generator.cpp 
 *
 * TODO: separate this file??
 * */
GaitGenerator::GaitGenerator(std::string method)
{
  if (method == "csv")
    throw std::invalid_argument( "Please provide filename for csv implementation" );
  /*
  else if ( method == "random")
    impl_ = static_cast<ImplPtr>(new RandImpl());
  else if ( method == "spline")
  {
    std::cout << "call spline constructor\n";
    impl_ = static_cast<ImplPtr>(new SplineImpl());
  }
  */
}

GaitGenerator::GaitGenerator(std::string method, std::string filename)
{
  if ( method == "csv" )
  {
    CsvImpl* csv = new CsvImpl();
    csv->set_csv_file( filename);
    impl_ = static_cast<ImplPtr>(csv);
  }
}

void GaitGenerator::next_step()
{
  if ( !is_valid() )
    throw std::invalid_argument( "GaitGenerator was not implemented." );
  impl_->next_step_impl();
}

void GaitGenerator::Impl::print_gait_impl()
{
  for (auto it = gait.begin(); it != gait.end(); ++it)
  {
    for (auto it2 = it->begin(); it2 != it->end(); ++it2)
      std::cout << *it2 << " ";
    std::cout << '\n';
  }
}

void GaitGenerator::CsvImpl::set_csv_file( std::string filename )
{
  std::string delimeter = " ";
  std::ifstream csvfile( filename );
  std::string line, token;

  csvfile.is_open();
  if ( csvfile.is_open() )
  {
    std::getline( csvfile, line );

    while ( std::getline( csvfile, line ) )
    {
      size_t last = 0, next = 0;
      for ( int i = 0; i < 7; ++i)
        next = line.find(delimeter, next+1);
      last = next + 1;

      std::vector<double> step;
      for ( int i = 0; i < 8; ++i)
      {
        next = line.find(delimeter, last);
        token = line.substr(last, next-last);
        step.push_back( cmd_to_rad(i, atof(token.c_str() ) ) );
        last = next + 1;
      }
      gait.push_back(step);
    }

    curr = gait.begin();
  }
}

void GaitGenerator::CsvImpl::next_step_impl( )
{
  std::stringstream ss;
  curr++;
  for ( int i = 0; i < 8; ++i )
  {
    std_msgs::Float64 cmd;
    cmd.data = (*curr)[i];
    (*ControllerPtr)[i].publish(cmd);
    ss << "Joint" << i << ":" << cmd.data << " ";
  }
  ROS_INFO_STREAM( ss.str() );
}

bool GaitGenerator::CsvImpl::is_over_impl( )
{ return curr == gait.end(); }

} // namespace spyndra

#endif
