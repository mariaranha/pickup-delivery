#include "mycolor.h"
int main(){
  ColorList();   cout << endl;
  ColorPrint();  cout << endl;
  cout << "Code of NoColor: " << ColorCode("NoColor") << endl<< endl;
  ColorInsert("MyNewColor","#00a1ff");
  cout << "Code of MyNewColor: " << ColorCode("MyNewColor") << endl;
  cout << "RGB of MyNewColor: " << ColorRGB("MyNewColor") << endl;
  cout << "RGB of color with code 148: " << ColorRGB(148) << endl;
  cout << "Name of color with code 148: " << ColorName(148) << endl<< endl;
  cout << "Code of Red: " << ColorCode("Red") << endl;
  cout << "Code of White: " << ColorCode("White") << endl;
  cout << "Code of Blue: " << ColorCode("Blue") << endl << endl;
  cout << "RGB of Brown: " << ColorRGB("Brown") << endl;
  cout << "RGB of Magenta: " << ColorRGB("Magenta") << endl;
  cout << "RGB of Cyan: " << ColorRGB("Cyan") << endl;
  cout << "RGB of Black: " << ColorRGB("Black") << endl;

  cout << endl << "List of visual distinct colors (" << SizeVisualDistinctColor() << ")" << endl;

  // using the NextVisualDistinctColor(code) function
  int code=FirstVisualDistinctColor();
  for (int i=0;i<SizeVisualDistinctColor();i++) {
    cout<<"Visual Distinct: "<<ColorName(code)<<" RGB: "<<ColorRGB(code)<<endl;
    code = NextVisualDistinctColor(code);
  }
  cout << endl << "List of visual distinct colors again (" << SizeVisualDistinctColor() << ")" << endl;

  // using the ith_VisualDistinctColor(i) function
  for (int i=0;i<SizeVisualDistinctColor();i++) {
    int code=ith_VisualDistinctColor(i);
    cout<<"Visual Distinct: "<<ColorName(code)<<" RGB: "<<ColorRGB(code)<<endl;
  }

  /*
  cout << endl << "List of some colorcodes" << endl;
  cout<<"NOCOLOR: "<<ColorCode("NoColor")<<endl;
  cout<<"WHITE: "<<ColorCode("White")<<endl;
  cout<<"BLACK: "<<ColorCode("Black")<<endl;
  cout<<"RED: "<<ColorCode("Red")<<endl;
  cout<<"GREEN: "<<ColorCode("Green")<<endl;
  cout<<"BLUE: "<<ColorCode("Blue")<<endl;
  cout<<"YELLOW: "<<ColorCode("Yellow")<<endl;
  cout<<"MAGENTA: "<<ColorCode("Magenta")<<endl;
  cout<<"CYAN: "<<ColorCode("Cyan")<<endl;
  cout<<"GRAY: "<<ColorCode("Gray")<<endl;
  cout<<"ORANGE: "<<ColorCode("Orange")<<endl;
  */
}
