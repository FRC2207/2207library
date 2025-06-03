package frc.robot.lib.leds;

import frc.robot.lib.leds.LedController;
/** This is a enum containing LED colors in the HSV format.
 * Colors must be pulled from this file to be compatible with the White Bear Lake LED Library.
 * 
 * If a color is not supported, 
 * please use the colorTest() mode and comment on the GitHub repository with the color recommendation (include web browser hue value)
 * Please keep in mind that these colors were tested with WS2812 light strands and may show up differently on other light strands. 
 */
public enum LedColor{
    LIGHT_RED,
    SALMON,
    RED,
    RED_ORANGE,
    PEACH,
    ORANGE,
    LIGHT_ORANGE,
    GOLD,
    LIGHT_YELLOW,
    YELLOW,
    SOFT_LIME,
    YELLOW_GREEN,
    EASTER_GREEN,
    LIME,
    LIGHT_GREEN,
    GREEN,
    AQUA,
    SEA_GREEN,
    SEA_BLUE,
    LIGHT_BLUE,
    TURQUOISE,
    BABY_BLUE,
    CORNFLOWER,
    SKY_BLUE,
    DARK_SKY_BLUE,
    BLUE,
    INDIGO,
    LIGHT_PURPLE,
    EASTER_PURPLE,
    PURPLE,
    BRIGHT_PURPLE,
    LIGHT_PINK,
    PINK,
    ROSE,
    MAGENTA,
    BROWN,
    WHITE,
    BLACK;

    public int hues() {
        switch (this) { // Values are divided by 2 because color pickers like Google are x/360, whereas
                        // WPILib is x/180
            
            case LIGHT_RED:
                return 0;
            case SALMON:
                return 0;
            case RED:
                return 0;
            case RED_ORANGE:
                return 6 / 2;
            case PEACH:
                return 10 / 2;
            case LIGHT_ORANGE:
                return 20 / 2;
            case ORANGE:
                return 10 / 2;
            case GOLD:
                return 30 / 2;
            case LIGHT_YELLOW:
                return 50 / 2;
            case YELLOW:
                return 50 / 2;
            case YELLOW_GREEN:
                return 76 / 2;
            case SOFT_LIME:
                return 76 / 2;
            case EASTER_GREEN:
                return 90 / 2;
            case LIME:
                return 90 / 2;
            case LIGHT_GREEN:
                return 110 / 2;
            case GREEN:
                return 110 / 2;
            case SEA_GREEN:
                return 140 / 2;
            case AQUA:
                return 140 / 2;
            case SEA_BLUE:
                return 160 / 2;
            case LIGHT_BLUE:
                return 180 / 2;
            case TURQUOISE:
                return 180 / 2;
            case BABY_BLUE:
                return 200 / 2;
            case CORNFLOWER:
                return 200 / 2;
            case SKY_BLUE:
                return 215 / 2;
            case DARK_SKY_BLUE:
                return 240 / 2;
            case BLUE:
                return 240 / 2;
            case INDIGO:
                return 260 / 2;
            case LIGHT_PURPLE:
                return 260 / 2;
            case EASTER_PURPLE:
                return 276 / 2;
            case PURPLE:
                return 276 / 2;
            case BRIGHT_PURPLE:
                return 290 / 2;
            case LIGHT_PINK:
                return 336 / 2;
            case PINK:
                return 336 / 2;
            case ROSE:
                return 356 / 2;
            case MAGENTA:
                return 356 / 2;
            case BROWN:
                return 20 / 2;
            case WHITE:
                return 0;
            case BLACK:
                return 0;
            default:
                return 0;
        }
    }

    public int saturation() {
        switch (this) {
            case LIGHT_RED:
                return 192;
            case SALMON:
                return 125;
            case PEACH:
                return 125;
            case LIGHT_ORANGE:
                return 125;
            case LIGHT_YELLOW:
                return 125;
            case SOFT_LIME:
                return 125;
            case EASTER_GREEN:
                return 125;
            case LIGHT_GREEN:
                return 125;
            case SEA_GREEN:
                return 125;
            case SEA_BLUE:
                return 125;
            case LIGHT_BLUE:
                return 125;
            case BABY_BLUE:
                return 125;
            case SKY_BLUE:
                return 125;
            case DARK_SKY_BLUE:
                return 125;
            case LIGHT_PURPLE:
                return 125;
            case EASTER_PURPLE:
                return 125;
            case LIGHT_PINK:
                return 125;
            case ROSE:
                return 125;
            case BROWN:
                return 70;
            case WHITE:
                return 0;
            case BLACK:
                return 0;
            default:
                return 255;
        }
    }

    public int value() {
        switch (this) {
            case BROWN:
                return 60;
            case BLACK:
                return 0;
            default:
                return LedController.ledBrightness;
        }
    }

}
