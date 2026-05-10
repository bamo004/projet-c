/*
 * =============================================================================
  * Ési | Programmation Avancée en C

 * Étudiant : BADR MOUNIRI | Pr. Tarik HOUICHIME
 * =============================================================================
  */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <string.h>


struct Drone {
    int   id;   /* Identifiant unique du drone   (  hadi tt )     */
    float x;    /* Coordonnée  spatiale axe X (mètres) */
    float y;    /* Coordonnée spatiale axe Y (mètres) */
    float z;    /* Coordonnée spatiale axe Z (mètres) */
};

/* ============================================================
 * RÉSULTAT DE LA PAIRE LA PLUS PROCHE
 * ============================================================ */
typedef struct {
    struct Drone *drone_a;   /* Pointeur vers le premier drone  */
    struct Drone *drone_b;   /* Pointeur vers le second drone   */
    float         distance;  /* Distance euclidienne minimale   */
} PairResult;

/* ============================================================
 * CONSTANTES
 * ============================================================ */
#define N_DRONES      10000
#define ESPACE_MAX    5000.0f   /* Volume aérien simulé (mètres) */

/* ============================================================
 * CALCUL DE DISTANCE EUCLIDIENNE 3D
 * ici on doit : Opère sur des pointeurs, jamais sur des indices
 * ============================================================ */
static inline float distance_euclidienne(const struct Drone *a,
                                         const struct Drone *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/* ============================================================
 * COMPARATEUR POUR TRI SUR L'AXE X
 * Utilisé par qsort — reçoit des pointeurs génériques
 * ============================================================ */
static int cmp_x(const void *pa, const void *pb)
{
    /* Déréférencement du pointeur générique vers struct Drone* */
    const struct Drone *a = *(const struct Drone **)pa;
    const struct Drone *b = *(const struct Drone **)pb;
    if (a->x < b->x) return -1;
    if (a->x > b->x) return  1;
    return 0;
}

/* ============================================================
 * COMPARATEUR POUR TRI SUR L'AXE Y (bande centrale)
 * ============================================================ */
static int cmp_y(const void *pa, const void *pb)
{
    const struct Drone *a = *(const struct Drone **)pa;
    const struct Drone *b = *(const struct Drone **)pb;
    if (a->y < b->y) return -1;
    if (a->y > b->y) return  1;
    return 0;
}

/* ============================================================
 * FORCE BRUTE — SOUS-ENSEMBLE DE TAILLE ≤ 3
 * Complexité O(1) (taille bornée par la constante 3)
 * Navigation 100% par arithmétique de pointeurs
 * ======================================= */
static PairResult brute_force(struct Drone **pp, int n)
{
    PairResult res;
    res.distance = FLT_MAX;
    res.drone_a  = NULL;
    res.drone_b  = NULL;

    struct Drone **pi = pp;                     /* pointeur itérateur i */
    struct Drone **fin = pp + n;

    while (pi < fin) {
        struct Drone **pj = pi + 1;             /* pointeur itérateur j */
        while (pj < fin) {
            float d = distance_euclidienne(*pi, *pj);
            if (d < res.distance) {
                res.distance = d;
                res.drone_a  = *pi;
                res.drone_b  = *pj;
            }
            pj++;                               /* avance pj d'une case */
        }
        pi++;                                   /* avance pi d'une case */
    }
    return res;
}

/* ============================================================
 * ANALYSE DE LA BANDE CENTRALE — Phase Combine
 * Trie les drones par Y dans la bande [-delta, +delta]
 * et vérifie au plus 7 voisins (preuve géométrique)
 * ============================================================ */
static PairResult bande_centrale(struct Drone **bande, int nb,
                                  PairResult meilleur)
{
    /* Tri de la bande selon l'axe Y via pointeurs */
    qsort(bande, (size_t)nb, sizeof(struct Drone *), cmp_y);

    struct Drone **pi = bande;
    struct Drone **fin = bande + nb;

    while (pi < fin) {
        /* pj parcourt au plus 7 éléments suivants (invariant géométrique) */
        struct Drone **pj = pi + 1;
        struct Drone **limite = pi + 8;        /* borne stricte */
        if (limite > fin) limite = fin;

        while (pj < limite) {
            /* Vérification de la contrainte Y avant calcul complet */
            float dy = (*pj)->y - (*pi)->y;
            if (dy >= meilleur.distance) break; /* tri Y permet ce break */

            float d = distance_euclidienne(*pi, *pj);
            if (d < meilleur.distance) {
                meilleur.distance = d;
                meilleur.drone_a  = *pi;
                meilleur.drone_b  = *pj;
            }
            pj++;
        }
        pi++;
    }
    return meilleur;
}

/* ============================================================
 * ALGORITHME DIVIDE & CONQUER — RÉCURSIF
 * Paramètres :
 *   pp  : tableau de pointeurs vers struct Drone, trié par X
 *   n   : nombre d'éléments dans ce sous-tableau
 * Retour   : PairResult contenant la paire la plus proche
 * ============================================================ */
static PairResult closest_pair_rec(struct Drone **pp, int n)
{
    /* --- CAS DE BASE --- */
    if (n <= 3)
        return brute_force(pp, n);

    /* --- DIVISER --- */
    int mid = n / 2;
    struct Drone *pivot = *(pp + mid);   /* drone médian (accès par pointeur) */
    float x_mid = pivot->x;

    /* Récursion gauche : pointeurs pp[0..mid-1] */
    PairResult gauche = closest_pair_rec(pp, mid);

    /* Récursion droite : pointeurs pp[mid..n-1] */
    PairResult droite = closest_pair_rec(pp + mid, n - mid);

    /* --- COMBINER : choisir le meilleur des deux sous-problèmes --- */
    PairResult meilleur = (gauche.distance <= droite.distance)
                          ? gauche : droite;
    float delta = meilleur.distance;

    /* --- CONSTRUIRE LA BANDE CENTRALE --- */
    /* Allocation temporaire du tableau de pointeurs de la bande */
    struct Drone **bande = (struct Drone **)malloc(
                                (size_t)n * sizeof(struct Drone *));
    if (!bande) {
        fprintf(stderr, "ERREUR CRITIQUE : malloc bande échoué\n");
        exit(EXIT_FAILURE);
    }

    int nb = 0;
    struct Drone **pc = pp;                /* pointeur curseur */
    struct Drone **fin = pp + n;

    /* Sélection par arithmétique de pointeurs — aucun crochet [] */
    while (pc < fin) {
        float dist_x = (*pc)->x - x_mid;
        if (dist_x < 0.0f) dist_x = -dist_x;  /* valeur absolue manuelle */
        if (dist_x < delta) {
            *(bande + nb) = *pc;               /* écriture via pointeur */
            nb++;
        }
        pc++;
    }

    /* --- PHASE COMBINE : analyse de la bande --- */
    meilleur = bande_centrale(bande, nb, meilleur);

    free(bande);
    return meilleur;
}

/* ============================================================
 * POINT D'ENTRÉE DE L'ALGORITHME PRINCIPAL
 * Prépare le tableau de pointeurs + tri initial
 * ============================================================ */
PairResult detecter_collision(struct Drone *essaim, int n)
{
    /* Allocation du tableau de pointeurs (vecteur d'indirection) */
    struct Drone **pp = (struct Drone **)malloc(
                            (size_t)n * sizeof(struct Drone *));
    if (!pp) {
        fprintf(stderr, "ERREUR CRITIQUE : malloc pointeurs échoué\n");
        exit(EXIT_FAILURE);
    }

    /* Initialisation par arithmétique de pointeurs */
    struct Drone **cur = pp;
    struct Drone  *src = essaim;
    struct Drone  *src_fin = essaim + n;

    while (src < src_fin) {
        *cur = src;   /* chaque case du tableau pointe vers un drone */
        cur++;
        src++;
    }

    /* Tri global sur X — O(n log n) */
    qsort(pp, (size_t)n, sizeof(struct Drone *), cmp_x);

    /* Appel récursif */
    PairResult res = closest_pair_rec(pp, n);

    free(pp);
    return res;
}

/* ============================================================
 * GÉNÉRATION DE DONNÉES DE TEST — Simulation Radar
 * Peuple l'essaim via arithmétique de pointeurs
 * ============================================================ */
static void generer_essaim(struct Drone *essaim, int n)
{
    srand(42u);   /* graine fixe pour reproductibilité */

    struct Drone *p   = essaim;
    struct Drone *fin = essaim + n;
    int id = 0;

    while (p < fin) {
        p->id = id;
        p->x  = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        p->y  = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        p->z  = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        p++;
        id++;
    }

    /* Injection d'une paire critique à distance connue */
    struct Drone *p_crit   = essaim + 100;   /* arithmétique de pointeurs */
    struct Drone *p_voisin = essaim + 101;

    p_crit->x = 1000.0f; p_crit->y = 1000.0f; p_crit->z = 500.0f;
    p_voisin->x = 1000.5f; p_voisin->y = 1000.2f; p_voisin->z = 500.1f;
}

/* ============================================================
 * MAIN — Orchestration du Système de Sécurité
 * ============================================================ */
int main(void)
{
    printf("=================================================\n");
    printf("  SYSTÈME DE DÉTECTION DE COLLISION — UAV SWARM \n");
    printf("  N = %d drones | Architecture O(n log² n)      \n", N_DRONES);
    printf("=================================================\n\n");

    /* --- ALLOCATION DYNAMIQUE DE L'ENTREPÔT CONTINU --- */
    struct Drone *essaim = (struct Drone *)malloc(
                               (size_t)N_DRONES * sizeof(struct Drone));
    if (!essaim) {
        fprintf(stderr, "ERREUR FATALE : Impossible d'allouer le tas.\n");
        return EXIT_FAILURE;
    }
    printf("[MÉMOIRE] Entrepôt alloué : %zu octets (%.2f Mo)\n\n",
           (size_t)N_DRONES * sizeof(struct Drone),
           (double)(N_DRONES * sizeof(struct Drone)) / (1024.0 * 1024.0));

    /* --- SIMULATION DES DONNÉES RADAR --- */
    generer_essaim(essaim, N_DRONES);
    printf("[RADAR]   Données de %d drones générées.\n\n", N_DRONES);

    /* --- DÉTECTION DE COLLISION --- */
    printf("[CALCUL]  Lancement de l'algorithme Divide & Conquer...\n");
    PairResult alerte = detecter_collision(essaim, N_DRONES);

    /* --- RAPPORT D'ALERTE --- */
    printf("\n[ALERTE]  ══===═══════════\n");
    printf("[ALERTE]  PAIRE CRITIQUE DÉTECTÉE !\n");
    printf("[ALERTE]  Drone A : ID=%-5d  (%.2f, %.2f, %.2f)\n",
           alerte.drone_a->id,
           alerte.drone_a->x, alerte.drone_a->y, alerte.drone_a->z);
    printf("[ALERTE]  Drone B : ID=%-5d  (%.2f, %.2f, %.2f)\n",
           alerte.drone_b->id,
           alerte.drone_b->x, alerte.drone_b->y, alerte.drone_b->z);
    printf("[ALERTE]  Distance minimale : %.6f mètres\n", alerte.distance);
    printf("[ALERTE]  MANŒUVRE D'ÉVITEMENT DÉCLENCHÉE.\n");
    printf("[ALERTE]  ═════════════════\n");

    /* --- LIBÉRATION SÉCURISÉE --- */
    free(essaim);
    essaim = NULL;   /* défensif : évite double-free */

    printf("\n[OK]      Mémoire libérée. Système opérationnel.\n");
    return EXIT_SUCCESS;
}
