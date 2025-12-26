# Midterm Project for Embedded System Design


[Music Player UI](https://youtu.be/KYRLsjRgu-M?feature=shared)


## Music Player UI Mockup

| Element      | Widget              | Description                              |
| ------------ | ------------------- | ---------------------------------------- |
| Album Art  (Done) | `lv_img`            | Show album image.      |
| Song Title  (Done)| `lv_label`          | Song title ticker.    |
| Artist Name  (Done)| `lv_label`          | Smaller gray text below the title.            |
| Progress Bar  (Done) | `lv_slider`         | Show playback progress.                   |
| Time labels  (Done) | `lv_label`          | “0:00” and “3:45” at both ends of the slider. |
| Buttons  (Done)  | `lv_btn` + `lv_img` | Play / Pause / Next / Prev buttons.       |
| Background  (Done)  | `lv_obj`            | Apply gradient or dark theme.             |

## Implementation Steps Completed

Sequence of UI features implemented (see `lvgl/examples/mainfunction.c`):

1. Gradient Background:

  - Create a radial gradient background with a white center fading to a grey edge.

2. Time Labels:

  - Convert the time label into mm:ss format in the slider event.
  - Place the elapsed time label on the left below the progress bar, and the total time label on the right below the progress bar.

3. Progress Bar (Slider):

  - Change the track color to medium gray and the indicator to light gray.
  - Hide the knob by reducing its opacity.

4. Control Buttons:

  - Add Prev / Pause / Next buttons under the slider.
  - Style each button as a circular shape with a transparent background and no border.
  - Cover the styled buttons with icons converted from PNG into C code.

5. Custom Cursor:

  - Add a custom cursor by converting the cursor icon from PNG into C code online using format `ARGB8888`, which allows opacity, and replace the cursor source file.

6. Improve Progress Bar (Slider): (Some times were not displayed.)

  - Set the slider range to 1000 to allow every second to be shown.
  - Modify the conversion code in the slider event.


7. Button Events:

  - Next and Prev reset the progress bar to 0 and update the time label.
  - Pause changes the opacity of the indicator.

8. Song Title Ticker:

  - If the song title string is longer than the slider box, it loops like a ticker.
  - Next and Pause reset the song title ticker.
  - Eight seconds between each loop.

9. Improve Overall UI:

  - Radial gradient background extends with the window.
  - Progress bar, labels, and buttons stay centered no matter how the window is resized.

10. Improve Code:

  - Reorder functions and add new comments.

11. Upgrade Pause Button:

  - The icon of the pause button now switches between play and pause.

12. Album Cover:

  - Album cover aligns to the top-middle of the progress bar.

13. Timer:

  - Progress bar advances over time.
  - Time labels update with the progress bar.

14. Chinese song title and artist name:

  - Allow Chinese song title and Artist name.

15. Final Commit?

  - Optimize and make code cleaner, and add more comment
