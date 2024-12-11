$(document).ready(function() {
    $('.search-container__form').on('submit', function(event) {
        event.preventDefault(); // 기본 폼 제출 방지

        var searchTerm = $('.search-container__input').val();

        $.ajax({
            url: '/search',
            method: 'GET',
            data: { query: searchTerm }, // 쿼리 파라미터로 검색어 전송
            success: function(response) {
                $('.search-container__result').empty(); // 이전 결과 비우기

                // 결과가 있을 경우
                if (Object.keys(response).length > 0) {
                    $('.search-container__result').append('<div>선반: ' + response.section_name + '</div>');
                    $('.search-container__result').append('<div>이름: ' + response.item_name + '</div>');
                    $('.search-container__result').append('<div>위치: ' + response.position + '</div>');
                    $('.search-container__result').append('<button class="search-container__start-guide-button">안내 시작</button>');
                } 
                // 결과가 없을 경우 메시지 표시
                else {
                    $('.search-container__result').append('<div>검색결과가 없습니다.</div>');
                }
            },
            error: function() {
                // 에러 처리
                $('.search-container__result').empty().append('<div>오류가 발생했습니다.</div>');
            }
        });
    });
});
