/**
 * @class Flock
 * handles flocking behavior
 */
class Flock {

  constructor(currentAgent) {
    this.currentAgent = currentAgent;
    this.wandertheta = 0;
  }

  /**
   * @method seek()
   * @param {*} target 
   * simple method to seek something
   */
  seek(target) {
    let desired = null;
    desired = Vector.sub(target, this.currentAgent.pos);
    desired.normalize();
    desired.mult(this.currentAgent.maxSpeed);
    let steer = Vector.sub(desired, this.currentAgent.vel);
    steer.limit(this.currentAgent.maxForce);
    return steer;
  }

  /**
   * @method flee()
   * @param {*} target 
   * simple method to flee something
   */
  flee(target) {
    let desired = null;
    let d = Vector.dist(this.currentAgent.pos, target);
    if (d < FLEE_RADIUS) {
      desired = Vector.sub(target, this.currentAgent.pos);
      desired.normalize();
      desired.mult(this.currentAgent.maxSpeed);
      let steer = Vector.sub(desired, this.currentAgent.vel);
      steer.limit(this.currentAgent.maxForce);
      return steer.mult(-1)
    } else {
      return new Vector(0, 0);
    }
  }

  /**
   * just a basic refator
   * @param {*} sum 
   */
  _returnSteer(sum) {
    sum.normalize();
    sum.mult(this.currentAgent.maxSpeed);
    let steer = Vector.sub(sum, this.currentAgent.vel);
    steer.limit(this.currentAgent.maxForce);
    return steer;
  }

  /**
   * @method wander()
   * not in used
   */
  wander() {
    let wanderR = 100;
    let wanderD = 80;
    let change = 0.1;
    this.wandertheta += -change + Math.random() * change;

    // Now we have to calculate the new location to steer towards on the wander circle
    let circleloc = this.currentAgent.vel.copy();
    circleloc.normalize();
    circleloc.mult(wanderD);
    circleloc.add(this.currentAgent.pos);

    let h = this.currentAgent.vel.heading();

    let circleOffSet = new Vector(wanderR * Math.cos(this.wandertheta + h), wanderR * Math.sin(this.wandertheta + h));
    let target = Vector.add(circleloc, circleOffSet);

    // SEEK (have to make the seek function generic)
    let desired = null;
    desired = Vector.sub(target, this.currentAgent.pos);
    desired.normalize();
    desired.mult(this.currentAgent.maxSpeed);
    let steer = Vector.sub(desired, this.currentAgent.vel);
    steer.limit(this.currentAgent.maxForce);
    return steer;
  }

  /**
   * @method calculerSeparation()
   * @param {Array} agents 
   * part of flocking system
   */
 calculerSeparation(agents) {
    // Distance minimale désirée entre l'agent courant et ses voisins
    // Ici fixée à 4 fois le rayon de l'agent
    let desiredseperation = 30;

    // Vecteur qui va accumuler les directions de "fuite" par rapport aux voisins trop proches
    let sum = new Vector();

    // Compteur des voisins considérés
    let count = 0;

    // Parcourt tous les agents
    for (let i = 0; i < agents.length; i++) {

      // Distance au carré entre l'agent courant et un autre agent
      let d = Vector.distSq(this.currentAgent.pos, agents[i].pos);

      // Vérifie que l'agent comparé n'est pas soi-même (d > 0)
      // et qu'il est plus proche que la distance de séparation souhaitée
      if ((d > 0) && (d < desiredseperation * desiredseperation)) {

        // Crée un vecteur de direction allant de l'autre agent vers l'agent courant
        // (c'est la direction dans laquelle on veut s'éloigner)
        let diff = Vector.sub(this.currentAgent.pos, agents[i].pos);

        // Normalise le vecteur (longueur = 1) pour garder seulement la direction
        diff.normalize();

        // Pondère par l'inverse de la distance au carré →
        // plus l'agent est proche, plus la force de répulsion est forte
        diff.div(d);

        // Ajoute cette contribution à la somme totale
        sum.add(diff);

        // Incrémente le compteur de voisins considérés
        count++;
      }
    }

    // Si au moins un voisin trop proche a été trouvé
    if (count > 0) {
      // Moyenne des vecteurs de répulsion calculés
      sum.div(count);

      // Retourne un vecteur de "steering" pour corriger la trajectoire
      // afin que l'agent s’éloigne des voisins trop proches
      return this._returnSteer(sum);
    }

    // Si aucun voisin proche, pas besoin de séparation → vecteur nul
    return new Vector(0, 0);
};


  /**
   * @method calculerAlignement()
   * @param {Array} agents 
   * part of flocking system
   */
  calculerAlignement(agents) {
    // Distance de voisinage considérée (rayon de perception des voisins)
    let neighbordist = 50;

    // Vecteur somme qui servira à accumuler les vitesses des voisins
    let sum = new Vector(0, 0);

    // Compteur du nombre de voisins pris en compte
    let count = 0;

    // Boucle sur tous les agents du système
    for (let i = 0; i < agents.length; i++) {

      // Calcul de la distance au carré entre l'agent courant et un autre agent
      // On utilise distSq (distance au carré) pour éviter la racine carrée, plus coûteuse en calcul
      let d = Vector.distSq(this.currentAgent.pos, agents[i].pos);

      // Vérifie que l'agent comparé n'est pas soi-même (d > 0)
      // et qu'il se situe à l'intérieur du rayon de voisinage (d < neighbordist²)
      if ((d > 0) && (d < neighbordist * neighbordist)) {

        // Ajoute la vélocité (direction + vitesse) du voisin à la somme
        sum.add(agents[i].vel);

        // Incrémente le nombre de voisins trouvés
        count++;
      }
    }

    // Si au moins un voisin a été trouvé
    if (count > 0) {
      // On calcule la vitesse moyenne des voisins en divisant la somme par leur nombre
      sum.div(count);

      // On retourne un vecteur de "steering" (ajustement de direction)
      // pour que l'agent tende à s'aligner avec la vitesse moyenne de ses voisins
      return this._returnSteer(sum);
    }

    // Si aucun voisin n'a été trouvé, pas de force d'alignement → retour d'un vecteur nul
    return new Vector(0, 0);
}



  /**
   * @method calculerCohesion()
   * @param {Array} agents 
   * part of flocking system
   */
  calculerCohesion(agents) {
    // distance maximale pour considérer un voisin
    let neighbordist = 30;

    // vecteur qui va accumuler les positions des voisins
    let sum = new Vector(0, 0);

    // compteur du nombre de voisins détectés
    let count = 0;

    // On parcourt tous les agents
    for (let i = 0; i < agents.length; i++) {

      // Calcul de la distance au carré entre l'agent courant et l'agent[i]
      // (distSq = distance au carré, plus rapide que sqrt pour comparer des distances)
      let d = Vector.distSq(this.currentAgent.pos, agents[i].pos);

      // Si la distance est > 0 (évite soi-même) ET < rayon d'influence²
      if ((d > 0) && (d < neighbordist * neighbordist)) {

        // Ajouter la position du voisin au vecteur somme
        // (on veut plus tard calculer la moyenne des positions)
        sum.add(agents[i].pos);

        // Incrémenter le compteur de voisins
        count++;
      }
    }

    // Si des voisins ont été trouvés
    if (count > 0) {
      // Diviser la somme par le nombre de voisins
      // -> on obtient la position moyenne = centre de masse des voisins
      sum.div(count);

      // Calcul du vecteur direction vers ce centre de masse
      // (on soustrait la position actuelle de l'agent)
      sum.sub(this.currentAgent.pos);

      // Convertir ce vecteur en une force de direction adaptée (via _returnSteer)
      return this._returnSteer(sum);
    }

    // Si aucun voisin trouvé → pas de force de cohésion
    return new Vector(0, 0);
}

}
