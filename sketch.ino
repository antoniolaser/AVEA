           
              
    }
  }
  // Pausa minima a fine giro (era 10 ms): la lancetta a 40 Hz arriva puntuale e il giro
  // lascia comunque spazio agli altri task (watchdog, Wi-Fi, BLE).
  delay(1);
}
