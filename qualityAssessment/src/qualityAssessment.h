#ifndef RenderWindowUISingleInheritance_H
#define RenderWindowUISingleInheritance_H
 
#include <vtkSmartPointer.h>
#include <QPushButton>
#include <QMainWindow>
 
// Forward Qt class declarations
class Ui_qualityAssessment;
 
class qualityAssessment : public QMainWindow
{
  Q_OBJECT
public:
 
  // Constructor/Destructor
  qualityAssessment(); 
  ~qualityAssessment() {};
 
public slots:
 void leftButton();
 void rightButton();
  virtual void slotExit();

private:
 
  // Designer form
  Ui_qualityAssessment *ui;
};
 
#endif
