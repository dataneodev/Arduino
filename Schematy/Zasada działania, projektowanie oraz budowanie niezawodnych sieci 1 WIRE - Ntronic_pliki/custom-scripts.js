jQuery(document).ready(function ($) {
    // Add smooth scrolling to all links
    $(".sfm-floating-menu a").on('click', function (event) {
        // Store hash
        var hash = this.hash;
        // Make sure this.hash has a value before overriding default behavior
        if (hash !== "" && $(hash).length > 0) {
            // Prevent default anchor click behavior
            event.preventDefault();

            // Using jQuery's animate() method to add smooth page scroll
            // The optional number (800) specifies the number of milliseconds it takes to scroll to the specified area
            $('html, body').animate({
                scrollTop: $(hash).offset().top
            }, 800, function () {

                // Add hash (#) to URL when done scrolling (default click behavior)
                window.location.hash = hash;
            });
        } // End if
    });
});