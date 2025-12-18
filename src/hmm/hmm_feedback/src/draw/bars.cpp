#include "hmm_feedback/bars.h"

namespace rosneuro {
  
bars::bars(void) {
  this->sub_neuro_ = this->nh_.subscribe("/hmm/neuroprediction", 1, &bars::on_received_neuro_data, this);
  this->sub_events_ = this->nh_.subscribe("/events/bus", 1, &bars::cb_events, this);
  this->user_quit_ = false;
  this->engine_ = new neurodraw::Engine("bars");
	this->engine_->on_keyboard(&bars::on_keyboard_event, this);
  this->current_probablility_ = std::vector<float>(2, 0.0f);

}

bars::~bars(void) {
  if (this->engine_ != nullptr) {
    delete this->engine_;
  }
}

std::vector<float> string2vector_converter(std::string msg){
    // If possible, always prefer std::vector to naked array
    std::vector<float> v;

    //msg.replace(msg.find(", "), 2, " ");

    // Build an istream that holds the input string
    std::istringstream iss(msg);

    // Iterate over the istream, using >> to grab floats
    // and push_back to store them in the vector
    std::copy(std::istream_iterator<float>(iss),
    std::istream_iterator<float>(),
    std::back_inserter(v));

    // Put the result on standard out
    //std::copy(v.begin(), v.end(),
    //std::ostream_iterator<float>(std::cout, ", "));
    //std::cout << "\n";
    return v;

}

bool bars::configure() {
  // I do not know why it is not working in a different way, I do not care
  std::string tmp_s = "1.0 1.0 1.0";
  
  ros::param::param<std::string>("~thresholds", tmp_s, tmp_s);

  std::vector<float> tmp_v = string2vector_converter(tmp_s);

  this->f_treshold_l_ = (float) tmp_v[0];
  this->f_treshold_c_ = (float) tmp_v[1];  
  this->f_treshold_r_ = (float) tmp_v[2];

  this->setup_scene();
  return true;
}

void bars::setup_scene(void) {
  this->probablility_l_ = new neurodraw::Rectangle(0.15f, 0.02f, true, neurodraw::Palette::blue);
  this->probablility_r_ = new neurodraw::Rectangle(0.15f, 0.02f, true, neurodraw::Palette::red);
  this->probablility_c_ = new neurodraw::Rectangle(0.15f, 0.02f, true, neurodraw::Palette::yellow);

  this->fixation_cross_ = new neurodraw::Cross(0.45f, 0.1f);

  this->bar_l_ = new neurodraw::Rectangle(0.15f, 1.1f, true, neurodraw::Palette::lightslategray);
  this->bar_r_ = new neurodraw::Rectangle(0.15f, 1.1f, true, neurodraw::Palette::lightslategray);
  this->bar_c_ = new neurodraw::Rectangle(0.15f, 1.1f, true, neurodraw::Palette::lightslategray);

  this->cbar_l_ = new neurodraw::Rectangle(0.17f, 1.12f, true, neurodraw::Palette::dimgray);
  this->cbar_r_ = new neurodraw::Rectangle(0.17f, 1.12f, true, neurodraw::Palette::dimgray);
  this->cbar_c_ = new neurodraw::Rectangle(0.17f, 1.12f, true, neurodraw::Palette::dimgray);

  this->treshold_l_ = new neurodraw::Rectangle(0.15f, 0.03f, true, neurodraw::Palette::lightgray);
  this->treshold_r_ = new neurodraw::Rectangle(0.15f, 0.03f, true, neurodraw::Palette::lightgray);
  this->treshold_c_ = new neurodraw::Rectangle(0.15f, 0.03f, true, neurodraw::Palette::lightgray);

  this->circle_l_ = new neurodraw::Circle(0.15f, true, neurodraw::Palette::blue);
  this->circle_r_ = new neurodraw::Circle(0.15f, true, neurodraw::Palette::red);
  this->circle_c_ = new neurodraw::Circle(0.15f, true, neurodraw::Palette::yellow);

  // New arrow
  this->arrow_l_ = new neurodraw::Arrow(0.35f, 0.35f, true, neurodraw::Palette::blue);
  this->arrow_r_ = new neurodraw::Arrow(0.35f, 0.35f, true, neurodraw::Palette::red);
  this->arrow_c_ = new neurodraw::Arrow(0.35f, 0.35f, true, neurodraw::Palette::yellow);

  this->treshold_l_->move(-1.0f, this->f_treshold_l_-0.5f); // TODO: check this subtraction
  this->treshold_r_->move( 1.0f, this->f_treshold_r_-0.5f);
  this->treshold_c_->move( 0.0f, this->f_treshold_c_-0.5f);

  this->probablility_l_->move(-1.0f, -0.5f);
  this->probablility_r_->move( 1.0f, -0.5f);
  this->probablility_c_->move( 0.0f, -0.5f);

  this->bar_l_->move(-1.0f, 0.0f);
  this->bar_r_->move( 1.0f, 0.0f);
  this->bar_c_->move( 0.0f, 0.0f);

  this->cbar_l_->move(-1.0f, 0.0f);
  this->cbar_r_->move( 1.0f, 0.0f);
  this->cbar_c_->move( 0.0f, 0.0f);


  this->circle_l_->move( 0.0f, 0.8f);
  this->circle_r_->move( 0.0f, 0.8f); 
  this->circle_c_->move( 0.0f, 0.8f); 

  // New Arrow
  
  this->arrow_c_->rotate(-90.0);
  this->arrow_r_->rotate(180.0);

  this->arrow_l_->move( 0.0f, 0.8f);
  this->arrow_r_->move( 0.0f, 0.8f);
  this->arrow_c_->move( 0.0f, 0.8f);

  /*this->engine_->add(this->probablility_l_);
  this->engine_->add(this->probablility_r_);
  this->engine_->add(this->probablility_c_);*/

  this->engine_->add(this->cbar_l_);
  this->engine_->add(this->cbar_r_);
  this->engine_->add(this->cbar_c_);

  this->engine_->add(this->bar_l_);
  this->engine_->add(this->bar_r_);
  this->engine_->add(this->bar_c_);

  this->engine_->add(this->treshold_l_);
  this->engine_->add(this->treshold_r_);
  this->engine_->add(this->treshold_c_);

  this->engine_->add(this->circle_l_);
  this->engine_->add(this->circle_r_);
  this->engine_->add(this->circle_c_);

  // New Arrow
  this->engine_->add(this->arrow_l_);
  this->engine_->add(this->arrow_r_);
  this->engine_->add(this->arrow_c_);

  this->engine_->add(this->fixation_cross_);

  this->engine_->add(this->probablility_l_);
  this->engine_->add(this->probablility_r_);
  this->engine_->add(this->probablility_c_);

  this->clear_scene();
}

void bars::clear_scene() {
  this->fixation_cross_->hide();
  this->circle_l_->hide();
  this->circle_r_->hide();
  this->circle_c_->hide();

  this->probablility_l_->hide();
  this->probablility_r_->hide();
  this->probablility_c_->hide();

  this->bar_l_->hide();
  this->bar_r_->hide();
  this->bar_c_->hide();

  this->cbar_l_->hide();
  this->cbar_r_->hide();
  this->cbar_c_->hide();

  this->treshold_l_->hide();
  this->treshold_r_->hide();
  this->treshold_c_->hide();

  this->arrow_l_->hide();
  this->arrow_r_->hide();
  this->arrow_c_->hide();
}

void bars::show_feedback() {
  this->probablility_l_->show();
  this->probablility_r_->show();
  this->probablility_c_->show();

  this->bar_l_->show();
  this->bar_r_->show();
  this->bar_c_->show();

  this->cbar_l_->show();
  this->cbar_r_->show();
  this->cbar_c_->show();

  this->treshold_l_->show();
  this->treshold_r_->show();
  this->treshold_c_->show();
}

void bars::show_fixation() {
  this->fixation_cross_->show();
}

void bars::show_task(Task t) {
  switch (t) {
  case Task::CENTER:
    this->arrow_c_->show();
    break;
  case Task::LEFT:
    this->arrow_l_->show();
    break;
  case Task::RIGHT:
    this->arrow_r_->show();
    break;
  }
}

void bars::cb_events(const rosneuro_msgs::NeuroEvent::ConstPtr& msg) {
  switch(msg->event) {
    case 786:
      this->clear_scene();
      this->show_fixation();
      break;
    case 781:
      // this->clear_scene();
      this->show_feedback();
      break;
    case 783:
      this->clear_scene();
      this->show_task(Task::CENTER);
      break;
    case 771:
      this->clear_scene();
      this->show_task(Task::RIGHT);
      break;
    case 773:
      this->clear_scene();
      this->show_task(Task::LEFT);
      break;
    case 897:
    case 898:
      // Hit and Miss
      this->clear_scene();
      break;
  }
}

void bars::on_received_neuro_data(const rosneuro_msgs::NeuroOutput& msg) {
  this->current_probablility_ = msg.softpredict.data;
  //ROS_INFO("%f %f %f ", this->current_probablility_[0],this->current_probablility_[1],this->current_probablility_[2]);
}

void bars::on_keyboard_event(const neurodraw::KeyboardEvent& event) {
  if(event.state == 0)
    return;
  switch(event.sym) {
    case neurodraw::EventKey::ESCAPE: {
      this->engine_->quit();
      this->user_quit_ = true;
      break;
    }
  }
}

void bars::update() {
  this->probablility_l_->move(-1.0f, this->current_probablility_[0] - 0.5f);
  this->probablility_c_->move( 0.0f, this->current_probablility_[1] - 0.5f);
  this->probablility_r_->move( 1.0f, this->current_probablility_[2] - 0.5f);
}

void bars::run(void) {

  this->show_feedback();

  ros::Rate r(100);
  while(ros::ok() & this->user_quit_ == false) {
    ros::spinOnce();
    this->update();
    r.sleep();
  }
}

}
