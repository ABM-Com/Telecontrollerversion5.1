#include "Persistence.tpp"
#define SerialMon Serial

void setup() {
  PersistenceClass persistence;
  persistence.init();
  persistence.clear(true);
  SerialMon.begin(9600);
  


}

void loop() {
  SerialMon.println("cleared");
}
