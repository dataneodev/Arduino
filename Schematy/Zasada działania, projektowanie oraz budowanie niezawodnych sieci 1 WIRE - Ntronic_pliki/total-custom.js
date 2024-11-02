/**
 * Total Custom JS
 *
 * @package Total
 *
 * Distributed under the MIT license - http://opensource.org/licenses/MIT
 */

jQuery(function ($) {

    /* Sticky Header */
    var hHeight = 0;
    var adminbarHeight = 0;
    if ($('body').hasClass('admin-bar')) {
        adminbarHeight = 32;
    }
    var $stickyHeader = $('.ht-header');
    if ($('.ht-sticky-header').length > 0 && $stickyHeader.length > 0) {
        hHeight = $stickyHeader.outerHeight();
        $pageWrapper = $('#ht-content');
        var hOffset = $stickyHeader.offset().top;

        var offset = hOffset - adminbarHeight;

        $stickyHeader.headroom({
            offset: offset,
            onTop: function () {
                $pageWrapper.css({
                    paddingTop: 0
                });
            },
            onNotTop: function () {
                $pageWrapper.css({
                    paddingTop: hHeight + 'px'
                });
            }
        });
    }

    if ($('#ht-bx-slider .ht-slide').length > 0) {
        $('#ht-bx-slider').owlCarousel({
            autoplay: true,
            items: 1,
            loop: true,
            nav: true,
            dots: false,
            autoplayTimeout: 7000,
            animateOut: 'fadeOut'
        });
    }

    $('.ht-testimonial-slider').owlCarousel({
        autoplay: true,
        items: 1,
        loop: true,
        nav: true,
        dots: false,
        autoplayTimeout: 7000,
        navText: ['<i class="fas fa-chevron-left" aria-hidden="true"></i>', '<i class="fas fa-chevron-right" aria-hidden="true"></i>']
    });

    $(".ht-logo-slider").owlCarousel({
        autoplay: true,
        items: 5,
        loop: true,
        nav: false,
        dots: false,
        autoplayTimeout: 7000,
        responsive: {
            0: {
                items: 2,
            },
            768: {
                items: 3,
            },
            979: {
                items: 4,
            },
            1200: {
                items: 5,
            }
        }
    });

    $('.ht-portfolio-image').nivoLightbox();

    var sf = $('.ht-menu > ul').superfish({
        delay: 500, // one second delay on mouseout
        animation: {opacity: 'show', height: 'show'}, // fade-in and slide-down animation
        speed: 'fast', // faster animation speed
    });

    $(window).resize(function () {
        if ($(window).width() < 1000) {
            sf.superfish('destroy');
            $('.ht-dropdown').removeClass('ht-opened');
        } else {
            sf.superfish('init');
        }
    }).resize();

    $('.ht-menu .menu-item-has-children > a').append('<button class="ht-dropdown"></button>');

    $('.ht-dropdown').on('click', function () {
        $(this).parent('a').next('ul').slideToggle();
        $(this).toggleClass('ht-opened');
        return false;
    })

    $('.ht-service-excerpt h5').click(function () {
        $(this).next('.ht-service-text').slideToggle();
        $(this).parents('.ht-service-post').toggleClass('ht-active');
    });

    $('.ht-service-icon').click(function () {
        $(this).next('.ht-service-excerpt').find('.ht-service-text').slideToggle();
        $(this).parent('.ht-service-post').toggleClass('ht-active');
    });

    $('.toggle-bar').click(function () {
        $(this).next('.ht-menu').slideToggle();
        totalKeyboardLoop($('.ht-main-navigation'));
        return false;
    });

    setTimeout(function () {
        $.stellar({
            horizontalScrolling: false,
            responsive: true,
        });
    }, 3000);

    $('.ht-team-counter-wrap').waypoint(function () {
        setTimeout(function () {
            $('.odometer1').html($('.odometer1').data('count'));
        }, 500);
        setTimeout(function () {
            $('.odometer2').html($('.odometer2').data('count'));
        }, 1000);
        setTimeout(function () {
            $('.odometer3').html($('.odometer3').data('count'));
        }, 1500);
        setTimeout(function () {
            $('.odometer4').html($('.odometer4').data('count'));
        }, 2000);
    }, {
        offset: 800,
        triggerOnce: true
    });

    if ($('.ht-sticky-header').length > 0) {
        var onpageOffset = 74;
    } else {
        onpageOffset = 0
    }

    $('.ht-sticky-header .ht-menu').onePageNav({
        currentClass: 'current',
        changeHash: false,
        scrollSpeed: 750,
        scrollThreshold: 0.1,
        scrollOffset: onpageOffset
    });

    // *only* if we have anchor on the url
    var anchorId = window.location.hash;
    anchorId = anchorId.replace('/', '');
    if ($(anchorId).length > 0) {
        $('html, body').animate({
            scrollTop: $(anchorId).offset().top - onpageOffset
        }, 1000);
    }

    $(window).scroll(function () {
        if ($(window).scrollTop() > 300) {
            $('#ht-back-top').removeClass('ht-hide');
        } else {
            $('#ht-back-top').addClass('ht-hide');
        }
    });

    $('#ht-back-top').click(function () {
        $('html,body').animate({scrollTop: 0}, 800);
    });

    if ($('.ht-portfolio-posts').length > 0) {

        var first_class = $('.ht-portfolio-cat-name:first').data('filter');
        $('.ht-portfolio-cat-name:first').addClass('active');

        var $container = $('.ht-portfolio-posts').imagesLoaded(function () {

            $container.isotope({
                itemSelector: '.ht-portfolio',
                filter: first_class
            });

            var elems = $container.isotope('getFilteredItemElements');

            elems.forEach(function (item, index) {
                if (index == 0 || index == 4) {
                    $(item).addClass('wide');
                    var bg = $(item).find('.ht-portfolio-image').attr('href');
                    $(item).find('.ht-portfolio-wrap').css('background-image', 'url(' + bg + ')');
                } else {
                    $(item).removeClass('wide');
                }
            });

            GetMasonary();

            setTimeout(function () {
                $container.isotope({
                    itemSelector: '.ht-portfolio',
                    filter: first_class,
                });
            }, 2000);

            $(window).on('resize', function () {
                GetMasonary();
            });

        });

        $('.ht-portfolio-cat-name-list').on('click', '.ht-portfolio-cat-name', function () {
            var filterValue = $(this).attr('data-filter');
            $container.isotope({filter: filterValue});

            var elems = $container.isotope('getFilteredItemElements');

            elems.forEach(function (item, index) {
                if (index == 0 || index == 4) {
                    $(item).addClass('wide');
                    var bg = $(item).find('.ht-portfolio-image').attr('href');
                    $(item).find('.ht-portfolio-wrap').css('background-image', 'url(' + bg + ')');
                } else {
                    $(item).removeClass('wide');
                }
            });

            GetMasonary();

            var filterValue = $(this).attr('data-filter');
            $container.isotope({filter: filterValue});

            $('.ht-portfolio-cat-name').removeClass('active');
            $(this).addClass('active');
        });

        function GetMasonary() {
            var winWidth = window.innerWidth;
            if (winWidth > 580) {

                $container.find('.ht-portfolio').each(function () {
                    var image_width = $(this).find('img').width();
                    if ($(this).hasClass('wide')) {
                        $(this).find('.ht-portfolio-wrap').css({
                            height: (image_width * 2) + 15 + 'px'
                        });
                    } else {
                        $(this).find('.ht-portfolio-wrap').css({
                            height: image_width + 'px'
                        });
                    }
                });

            } else {
                $container.find('.ht-portfolio').each(function () {
                    var image_width = $(this).find('img').width();
                    if ($(this).hasClass('wide')) {
                        $(this).find('.ht-portfolio-wrap').css({
                            height: (image_width * 2) + 8 + 'px'
                        });
                    } else {
                        $(this).find('.ht-portfolio-wrap').css({
                            height: image_width + 'px'
                        });
                    }
                });
            }
        }

    }

    var totalKeyboardLoop = function (elem) {

        var tabbable = elem.find('select, input, textarea, button, a').filter(':visible');

        var firstTabbable = tabbable.first();
        var lastTabbable = tabbable.last();
        /*set focus on first input*/
        firstTabbable.focus();

        /*redirect last tab to first input*/
        lastTabbable.on('keydown', function (e) {
            if ((e.which === 9 && !e.shiftKey)) {
                e.preventDefault();
                firstTabbable.focus();
            }
        });

        /*redirect first shift+tab to last input*/
        firstTabbable.on('keydown', function (e) {
            if ((e.which === 9 && e.shiftKey)) {
                e.preventDefault();
                lastTabbable.focus();
            }
        });

        /* allow escape key to close insiders div */
        elem.on('keyup', function (e) {
            if (e.keyCode === 27) {
                elem.hide();
            }
            ;
        });
    };

});

