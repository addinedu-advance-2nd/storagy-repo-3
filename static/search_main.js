$(document).ready(function() {
    $('.search-container__form').on('submit', function(event) {
        event.preventDefault(); // 기본 폼 제출 방지

        var searchTerm = $('.search-container__input').val();

        $.ajax({
            url: '/search',
            method: 'GET',
            data: { query: searchTerm }, // 쿼리 파라미터로 검색어 전송
            success: function(response) {
                $('.result-container').empty(); // 이전 결과 비우기

                // 결과가 있을 경우
                if (Object.keys(response).length > 0) {
                    // 테이블 생성
                    let table = `
                        <table class="result-container__table">
                            <thead>
                                <tr>
                                    <th>선반</th>
                                    <th>이름</th>
                                    <th>위치</th>
                                </tr>
                            </thead>
                            <tbody>
                                <tr>
                                    <td>${response.section_name}</td>
                                    <td>${response.item_name}</td>
                                    <td>${response.position}</td>
                                </tr>
                            </tbody>
                        </table>
                        <button class="result-container__start-guide-button">안내<br>시작</button>`;
                    $('.result-container').append(table);
                } 
                // 결과가 없을 경우 메시지 표시
                else {
                    $('.result-container').append('<div class="result-container__message">검색결과가 없습니다.</div>');
                }
            },
            error: function() {
                // 에러 처리
                $('.result-container').empty().append('<div class="result-container__message">오류가 발생했습니다.</div>');
            }
        });
    });
});
