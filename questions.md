* implementace A*? (je vubec potreba, zatim jsou to vzdy stromy)
* posuny po vypocitane trajektorii - pres pozici nebo pres rychlosti(potreba vytvorit novy state space -> drahe)?
   * pro dynamicky stav - pri pozici - potreba znat rychlosti vsech dynamickych objektu a upravit tak pozici
     * problem s zmenami rychlosti, vse se musi prepocitavat, posila se mnoho dat 
   * pro dynamicky stav - pri rychlostech - potreba v kazdem nodu udrzovat kopii celeho state spacu
* local planner rotaci - otoceni po smeru/ proti smeru muze ovlivnit povoleni (jak potom postupovat ve 3D?)
* nestabilni fyzika - kdyz chci zreplikovat trasu pomoci seznamu waypointu, kde kazdy ma vektor rychlosti a cas do zmeny waypointu,
    tak se odchylim od trajektorie -> potreba korigovat natvrdo napsanou pozici